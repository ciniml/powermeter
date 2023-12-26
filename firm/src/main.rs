use std::{time::Duration, str::FromStr, net::Ipv4Addr, thread::{JoinHandle, self}, sync::{Arc, Mutex}, f32::consts::E, cell::RefCell, io::BufWriter};

use embedded_svc::{wifi::{AuthMethod, ClientConfiguration, Configuration, AccessPointConfiguration, Protocol, Wifi}, ipv4, http::Method, io::Write};

use esp_idf_svc::{hal::{peripherals::Peripherals, adc::{AdcDriver, attenuation, AdcChannelDriver, config::Config}, timer, delay::{NON_BLOCK, self, BLOCK}, gpio::{Pull, PinDriver}, modem::WifiModemPeripheral, task::{thread::ThreadSpawnConfiguration, block_on}, cpu::Core}, eventloop::EspSystemEventLoop, wifi::{BlockingWifi, EspWifi, WifiDeviceId, WifiDriver, WifiEvent}, netif::{EspNetif, NetifStack, NetifConfiguration, IpEvent}, http::client::EspHttpConnection};
use esp_idf_svc::hal::delay::TickType;
use esp_idf_svc::hal::task::queue::Queue;
use esp_idf_svc::nvs::*;

use log::{info, debug, error};

#[derive(Debug, Clone)]
struct MeasurementConfig {
    pub upload_interval_seconds: u32,
    pub upload_deploy_id: std::string::String,
    pub upload_device_id: std::string::String,
}

#[derive(Debug, Clone, Copy, Default)]
struct MeasuredData {
    pub current_arms: f32,
    pub power_w: f32,
    pub freq_hz: f32,
}

static MEASUREMENT_CONFIG: Mutex<RefCell<Option<MeasurementConfig>>> = Mutex::new(RefCell::new(None));

#[derive(Debug, Clone, Copy)]
struct SmartConfigResult([u8; 32], [u8; 64], Option<[u8; 6]>);

static mut SMART_CONFIG_QUEUE: Option<Queue<SmartConfigResult>> = None;
static mut MEASURED_DATA_QUEUE: Option<Queue<MeasuredData>> = None;

/**
 * @brief      SmartConfig event handler
 * @param[in]  arg          context
 * @param[in]  event_base   event base ID
 * @param[in]  event_id     event ID
 * @param[in]  event_data   event data
 */
pub unsafe extern "C" fn sc_event_handler(arg: *mut core::ffi::c_void, event_base: esp_idf_svc::sys::esp_event_base_t, event_id: i32, event_data: *mut core::ffi::c_void) {
    if event_base != esp_idf_svc::sys::SC_EVENT {
        return;
    }

    let queue = if let Some(queue) = SMART_CONFIG_QUEUE.as_ref() { queue } else { return; };
    static mut LAST_CONFIG: Option<SmartConfigResult> = None;
    let event_id = event_id as esp_idf_svc::sys::smartconfig_event_t;
    match event_id {
        esp_idf_svc::sys::smartconfig_event_t_SC_EVENT_SCAN_DONE => {
            info!("SmartConfig: Scan Done.");
        },
        esp_idf_svc::sys::smartconfig_event_t_SC_EVENT_FOUND_CHANNEL => {
            info!("SmartConfig: Found Channel.");
        },
        esp_idf_svc::sys::smartconfig_event_t_SC_EVENT_GOT_SSID_PSWD => {
            let event_data = &*(event_data as *const esp_idf_svc::sys::smartconfig_event_got_ssid_pswd_t);
            let ssid_len = event_data.ssid.iter().position(|&c| c == 0).unwrap_or(event_data.ssid.len());
            let password_len = event_data.password.iter().position(|&c| c == 0).unwrap_or(event_data.password.len());
            let ssid = core::str::from_utf8_unchecked(&event_data.ssid[..ssid_len]);
            let password = core::str::from_utf8_unchecked(&event_data.password[..password_len]);

            info!("SmartConfig: Got SSID: {}", ssid);
            debug!("SmartConfig: Got SSID: {} Password: {}", ssid, password);
            let mut wifi_config: esp_idf_svc::sys::wifi_config_t = Default::default();
            wifi_config.sta.ssid = event_data.ssid;
            wifi_config.sta.password = event_data.password;
            wifi_config.sta.bssid = event_data.bssid;
            wifi_config.sta.bssid_set = event_data.bssid_set;
            esp_idf_svc::sys::esp_wifi_disconnect();

            // Connect to the AP (required to send ACK to SmartConfig host)
            let err = esp_idf_svc::sys::esp_wifi_set_config(esp_idf_svc::sys::wifi_interface_t_WIFI_IF_STA, &mut wifi_config);
            if err != esp_idf_svc::sys::ESP_OK {
                panic!("Failed to set WiFi config: {:?}", err);
            }
            let err = esp_idf_svc::sys::esp_wifi_connect();
            if err != esp_idf_svc::sys::ESP_OK {
                panic!("Failed to connect to WiFi: {:?}", err);
            }

            LAST_CONFIG = Some(SmartConfigResult(event_data.ssid, event_data.password, if event_data.bssid_set { Some(event_data.bssid) } else { None }));
        },
        esp_idf_svc::sys::smartconfig_event_t_SC_EVENT_SEND_ACK_DONE => {
            info!("SmartConfig: Send Ack Done.");
            if let Some(config) = LAST_CONFIG.take() {
                queue.send_back(config, TickType::new(NON_BLOCK).ticks()).ok();
            }
        },
        _ => {
            info!("SmartConfig: Unknown event: {}", event_id);
        }
    }
}

fn run_wifi_task<M: WifiModemPeripheral + 'static>(modem: M, nvs: EspNvsPartition<NvsDefault>, sys_loop: EspSystemEventLoop, setup_mode: bool) -> (JoinHandle<()>, [u8; 6]) {
    unsafe {
        SMART_CONFIG_QUEUE = Some(Queue::new(1));
    }
    let measured_data_queue = unsafe {
        MEASURED_DATA_QUEUE = Some(Queue::new(60));
        MEASURED_DATA_QUEUE.as_ref().unwrap()
    };
    let delay = delay::Delay::new_default();
    let wifi = WifiDriver::new(modem, sys_loop.clone(), Some(nvs.clone())).unwrap();
    let wifi_ap_mac = wifi.get_mac(WifiDeviceId::Ap).unwrap();
    let handle = std::thread::Builder::new().name("WiFi".into()).stack_size(20480).spawn(move || {
        // Initialize WiFi STA.
        let wifi = EspWifi::wrap(wifi).unwrap();
        let mut wifi = BlockingWifi::wrap(
            wifi,
            sys_loop.clone(),
        ).unwrap();
        // Conifgure SmartConfig event handler.
        unsafe {
            esp_idf_svc::sys::esp_event_handler_register(
                esp_idf_svc::sys::SC_EVENT,
                esp_idf_svc::sys::ESP_EVENT_ANY_ID,
                Some(sc_event_handler),
                core::ptr::null_mut(),
            );
        }

        let mut closure = || -> anyhow::Result<()> {
            wifi.start()?;
        
            if setup_mode {
                // Setup mode
                info!("Starting setup mode");
                let err = unsafe { esp_idf_svc::sys::esp_smartconfig_set_type(esp_idf_svc::sys::smartconfig_type_t_SC_TYPE_ESPTOUCH) };
                if err != esp_idf_svc::sys::ESP_OK {
                    panic!("Failed to set SmartConfig type: {:?}", err);
                }
        
                // Set to STA mode
                wifi.set_configuration(&Configuration::Client(ClientConfiguration {
                    ssid: "".into(),
                    password: "".into(),
                    bssid: None,
                    auth_method: AuthMethod::WPA2WPA3Personal,
                    channel: None,
                }))?;
        
                let config = esp_idf_svc::sys::smartconfig_start_config_t {
                    enable_log: true,
                    esp_touch_v2_enable_crypt: false,
                    esp_touch_v2_key: core::ptr::null_mut(),
                };
                let err = unsafe { esp_idf_svc::sys::esp_smartconfig_start(&config) };
                if err != esp_idf_svc::sys::ESP_OK {
                    panic!("Failed to start SmartConfig: {:?}", err);
                }
                info!("Setup mode started");
                let (_config, _) = unsafe { SMART_CONFIG_QUEUE.as_ref().unwrap().recv_front(BLOCK).unwrap() };
                unsafe { esp_idf_svc::sys::esp_smartconfig_stop() };
                    
            } else {
                let mut wifi_configuration = wifi.get_configuration()?;
                info!("Using saved WiFi configuration: {:?}", wifi_configuration);
            }

            let mut client = embedded_svc::http::client::Client::wrap(EspHttpConnection::new(&esp_idf_svc::http::client::Configuration {
                buffer_size: Some(4096),
                buffer_size_tx: Some(4096),
                timeout: Some(Duration::from_secs(5)),
                crt_bundle_attach: Some(esp_idf_svc::sys::esp_crt_bundle_attach),
                 ..Default::default()
            })?);
            let headers = [("accept", "text/plain")];
            loop {
                info!("Connecting to WiFi");
                wifi.connect()?;
                info!("Connected to WiFi");
                wifi.wait_netif_up()?;
                
                // Discard any pending data
                while let Some(_) = measured_data_queue.recv_front(NON_BLOCK) {}
                // Post measured data while connection is active.
                while wifi.is_connected()? {
                    if let Some((data, _)) = measured_data_queue.recv_front(TickType::from(Duration::from_millis(1000)).ticks()) {
                        let post_string = if let Ok(config) = MEASUREMENT_CONFIG.lock() {
                            let config = config.borrow();
                            let config = config.as_ref().unwrap();
                            let deploy_id = &config.upload_deploy_id;
                            let device_id = &config.upload_device_id;
                            let power_w = if 45.0f32 <= data.freq_hz && data.freq_hz <= 65.0f32 {
                                data.power_w
                            } else {
                                0.0f32
                            };
                            let post_string = format!("https://script.google.com/macros/s/{}/exec?UniqueID={}&current_raw={}&power_raw={}&power={}&frequency={}", deploy_id, device_id, data.current_arms, data.power_w, power_w, data.freq_hz);
                            Some(post_string)
                        } else {
                            None
                        };

                        if let Some(post_string) = post_string {
                            info!("GET: {}", &post_string);
                            const MAX_TRIALS: usize = 3;
                            for trial in 0..MAX_TRIALS {
                                let result = client.request(Method::Get, &post_string, &headers)
                                    .and_then(|request| {
                                        match request.submit() {
                                            Ok(response) => {
                                                info!("Response: status={}", response.status());
                                                Ok(())
                                            },
                                            Err(e) => {
                                                error!("Failed to post measured data: {:?}", e);
                                                Err(e)
                                            },
                                        }
                                    });
                                if let Ok(_) = result {
                                    break;
                                } else {
                                    info!("Retrying in 1 second ({}/{})", trial+1, MAX_TRIALS);
                                    delay.delay_ms(1000);
                                }
                            }
                        }
                    }    
                }
            }
        }; 
        loop {
            closure().unwrap_or_else(|e| {
                error!("Error: {:?}", e);
                info!("Restarting WiFi task");
            });
            delay.delay_ms(1000);
        }
    }).unwrap();
    (handle, wifi_ap_mac)
}

fn main() -> anyhow::Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    // Initialize peripherals
    let peripherals = Peripherals::take()?;

    // Initialize ADC
    let mut adc = AdcDriver::new(peripherals.adc1, &Config::new().calibration(true))?;
    let mut adc_pin: AdcChannelDriver<{ attenuation::DB_6}, _> = AdcChannelDriver::new(peripherals.pins.gpio8)?;
    const SAMPLES_PER_SECOND: usize = 6000;
    const OFFSET_MV: i16 = 1100;

    // Initialize GPIOs
    let mut led = PinDriver::output(peripherals.pins.gpio35)?;
    let mut button = PinDriver::input(peripherals.pins.gpio41)?;
    button.set_pull(Pull::Up)?;

    static mut NVS_DEFAULT_PARTITION: Option<EspDefaultNvsPartition> = None;
    unsafe { NVS_DEFAULT_PARTITION = Some(EspDefaultNvsPartition::take()?); }
    let nvs_default_partition = unsafe { NVS_DEFAULT_PARTITION.as_ref().unwrap() };

    let sys_loop = EspSystemEventLoop::take()?;
    let mut measure_nvs = match EspNvs::new(nvs_default_partition.clone(), "measure", false) {
        Ok(measure_nvs) => {
            info!("Got namespace {:?} from default partition", "measure");
            Some(measure_nvs)
        }
        Err(e) => {
            error!("Could't get namespace {:?}", e);
            None
        },
    };
    // Initialize configuration
    let measurement_config = measure_nvs.map(|measure_nvs|{
        let mut upload_deploy_id_buffer = std::vec::Vec::new();
        upload_deploy_id_buffer.resize(128, 0);
        let mut upload_device_id_buffer = std::vec::Vec::new();
        upload_device_id_buffer.resize(128, 0);
        let mut config = MeasurementConfig {
            upload_interval_seconds: measure_nvs.get_u32("u_interval").ok().flatten().unwrap_or(60),
            upload_deploy_id: measure_nvs.get_str("u_deploy_id", &mut upload_deploy_id_buffer).ok().flatten().unwrap_or("").trim_end_matches(|c| c == '\0').into(),
            upload_device_id: measure_nvs.get_str("u_device_id", &mut upload_device_id_buffer).ok().flatten().unwrap_or("powermeter").trim_end_matches(|c| c == '\0').into(),
        };
        info!("Got config: {:?}", config);
        config
    }).unwrap_or_else(|| {
        info!("Using default config");
        MeasurementConfig {
            upload_interval_seconds: 60,
            upload_deploy_id: String::from(""),
            upload_device_id: String::from("powermeter"),
        }
    });
    MEASUREMENT_CONFIG.lock().unwrap().replace(Some(measurement_config));

    // Run WiFi task
    let (_wifi_task, wifi_mac) = run_wifi_task(peripherals.modem, nvs_default_partition.clone(), sys_loop.clone(), button.is_low());
    let measured_data_queue = unsafe { MEASURED_DATA_QUEUE.as_ref().unwrap() };

    std::thread::sleep(Duration::from_millis(100));

    // mDNS
    let mut mdns = esp_idf_svc::mdns::EspMdns::take()?;
    mdns.set_hostname("powermeter")?;
    mdns.set_instance_name(format!("powermeter-{:02X}{:02X}{:02X}", wifi_mac[3], wifi_mac[4], wifi_mac[5]))?;
    mdns.add_service(None, "_http", "_tcp", 80, &[])?;

    // Run HTTP server
    let _server = {
        let nvs = nvs_default_partition.clone();

        let mut server = esp_idf_svc::http::server::EspHttpServer::new(&esp_idf_svc::http::server::Configuration {
            http_port: 80,
            max_uri_handlers: 4,
            ..Default::default()
        })?;
        server.fn_handler("/", Method::Get, |request| {
            request.into_ok_response()?
                .write_all("PowerMeter configuration server.".as_bytes())?;
            Ok(())
        })?;
        server.fn_handler("/config", Method::Get, |request| {
            let body = {
                let guard = MEASUREMENT_CONFIG.lock()?;
                let config = guard.borrow();
                let config = config.as_ref().unwrap();
                format!(include_str!("config.html"), config.upload_interval_seconds, config.upload_deploy_id, config.upload_device_id)
            };
            request.into_response(200, Some("OK"), &[("Content-Type", "text/html")])?
                .write_all(body.as_bytes())?;
            Ok(())
        })?;
        server.fn_handler("/config", Method::Post, |mut request| {
            let mut config = MEASUREMENT_CONFIG.lock()?.borrow().as_ref().unwrap().clone();
            let mut body = Vec::new();
            body.resize(1024, 0);
            let mut bytes_read = 0;
            while let Ok(bytes) = request.read(&mut body[bytes_read..]) {
                if bytes < body.len() - bytes_read {
                    bytes_read += bytes;
                    body.resize(bytes_read, 0);
                    break;
                }
                bytes_read += bytes;
                body.resize(bytes_read + 1024, 0)
            }
            for (key, value) in url::form_urlencoded::parse(&body) {
                let key = match &key {
                    std::borrow::Cow::Borrowed(key) => key,
                    std::borrow::Cow::Owned(key) => key.as_str(),
                };
                match key {
                    "upload_interval" => {
                        if let Ok(upload_interval_seconds) = u32::from_str(&value) {
                            config.upload_interval_seconds = upload_interval_seconds;
                        }
                    },
                    "upload_deploy_id" => {
                        config.upload_deploy_id = value.into_owned();
                    },
                    "upload_device_id" => {
                        config.upload_device_id = value.into_owned();
                    },
                    _ => {
                        info!("Unknown key: {}", key);
                    },
                }
            }
            info!("New config: {:?}", config);
            MEASUREMENT_CONFIG.lock()?.replace(Some(config.clone()));
            match EspNvs::new(nvs_default_partition.clone(), "measure", true) {
                Ok(mut measure_nvs) => {
                    info!("Got namespace {:?} from default partition", "wifi");
                    measure_nvs.set_u32("u_interval", config.upload_interval_seconds)?;
                    measure_nvs.set_str("u_deploy_id", &config.upload_deploy_id)?;
                    measure_nvs.set_str("u_device_id", &config.upload_device_id)?;
                }
                Err(e) => panic!("Could't get namespace {:?}", e),
            };
            let body = format!(include_str!("config.html"), config.upload_interval_seconds, config.upload_deploy_id, config.upload_device_id);
            request.into_response(200, Some("OK"), &[("Content-Type", "text/html")])?
                .write_all(body.as_bytes())?;
            Ok(())
        })?;
        server
    };

    // Initialize Queue
    static mut ADC_QUEUE: Option<Queue<i16>> = None;
    unsafe {
        ADC_QUEUE = Some(Queue::new(SAMPLES_PER_SECOND/5));
    }

    
    // Spawn ADC sampling task
    ThreadSpawnConfiguration {
        name: Some(b"ADC\0"),
        stack_size: 8192,
        priority: 23,                   // ADC sampling is the highest priority task
        pin_to_core: Some(Core::Core1), // Run on core 1 to avoid jitter caused by WiFi interrupt.
        ..Default::default()
    }
    .set()
    .unwrap();
    let _adc_thread = std::thread::Builder::new().spawn(move || {
        || -> anyhow::Result<()> {
            // Initialize Timer
            let timer_config = timer::config::Config::new().auto_reload(true).divider(80);
            let mut timer = timer::TimerDriver::new(peripherals.timer00, &timer_config)?;
            timer.set_alarm(timer.tick_hz() / (SAMPLES_PER_SECOND as u64))?;
            timer.enable_interrupt()?;
            timer.enable_alarm(true)?;
            timer.enable(true)?;

            loop {
                block_on(timer.wait());
                timer.reset_wait();
                let value = adc.read(&mut adc_pin).unwrap() as i16;
                if let Some(queue) = unsafe{ &ADC_QUEUE } {
                    queue.send_back(value, TickType::new(NON_BLOCK).ticks()).ok();
                }
            }
        }().unwrap();
    });

    let mut number_of_samples = 0;
    let mut accumulated_squared = 0u64;
    let queue = unsafe { ADC_QUEUE.as_ref().unwrap() };
    
    const AVERAGING_SAMPLES: usize = 10;
    let mut average_queue = heapless::spsc::Queue::<i16, AVERAGING_SAMPLES>::new();
    let mut average_accumulator = 0i32;

    const OFFSET_DETECTION_AVERAGING_SAMPLES: usize = SAMPLES_PER_SECOND;
    static mut OFFSET_AVERAGE_QUEUE: heapless::spsc::Queue::<i16, OFFSET_DETECTION_AVERAGING_SAMPLES> = heapless::spsc::Queue::<i16, OFFSET_DETECTION_AVERAGING_SAMPLES>::new();
    let offset_average_queue = unsafe { &mut OFFSET_AVERAGE_QUEUE };
    let mut offset_average_accumulator = 0i32;
    let mut offset_mv = OFFSET_MV;

    let mut zero_cross_interval = 0;
    let mut zero_cross_count = 0;
    let mut zero_cross_accumulator = 0;
    let mut last_is_positive = None;

    let mut measurement_counter = 0;
    let mut measured_data: MeasuredData = Default::default();

    loop {
        if let Some((adc_voltage_mv, _)) = queue.recv_front(TickType::from(Duration::from_millis(1000)).ticks()) {
            let offset_centered_mv = adc_voltage_mv - OFFSET_MV;
            
            // Moving average with AVERAGING_SAMPLES window.
            average_accumulator += offset_centered_mv as i32;
            if !average_queue.is_full() {
                average_queue.enqueue(offset_centered_mv).ok();
                continue;
            }
            average_accumulator -= average_queue.dequeue().unwrap() as i32;
            average_queue.enqueue(offset_centered_mv).ok();
            let averaged_value = average_accumulator / AVERAGING_SAMPLES as i32;
            let offset_centered_mv = offset_centered_mv as i32;

            // Offset detection
            offset_average_accumulator += adc_voltage_mv as i32;
            offset_mv = if !offset_average_queue.is_full() {
                offset_average_queue.enqueue(adc_voltage_mv).ok();
                OFFSET_MV
            } else {
                offset_average_accumulator -= offset_average_queue.dequeue().unwrap() as i32;
                offset_average_queue.enqueue(adc_voltage_mv).ok();
                (offset_average_accumulator / OFFSET_DETECTION_AVERAGING_SAMPLES as i32) as i16
            };

            // Zero crossing detection (only rising edge, negative to positive transition)
            let is_positive = averaged_value > 0;
            let zero_crossed = match (last_is_positive, is_positive) {
                (Some(false), true) => true,
                _ => false,
            };
            last_is_positive = Some(is_positive);
            if zero_crossed && zero_cross_interval >= (SAMPLES_PER_SECOND / 60 / 2) {
                zero_cross_count += 1;
                zero_cross_accumulator += zero_cross_interval;
                zero_cross_interval = 0;
            } else {
                zero_cross_interval = zero_cross_interval.saturating_add(1);
            }

            accumulated_squared += (offset_centered_mv * offset_centered_mv) as u64;
            number_of_samples += 1;
            if number_of_samples == SAMPLES_PER_SECOND as u32 {
                let average_zero_cross_interval = if zero_cross_accumulator > 0 && zero_cross_count > 0 { Some((SAMPLES_PER_SECOND as f32) * (zero_cross_count as f32) / (zero_cross_accumulator as f32)) } else { None };
                let mean_squared_mv = (accumulated_squared / number_of_samples as u64) as f32;
                let rms_v = mean_squared_mv.sqrt() * 1.0e-3f32;   
                let current_arms = rms_v*(3000.0f32*1.0e-2f32);
                let power_w = current_arms * 100.0f32;
                let freq_hz = average_zero_cross_interval.unwrap_or_default();
                log::info!("offset: {}, RMS: {}, current: {}, power: {}, freq: {}", offset_mv, rms_v, current_arms, power_w, freq_hz);
                
                number_of_samples = 0;
                accumulated_squared = 0;
                zero_cross_accumulator = 0;
                zero_cross_count = 0;

                measured_data.current_arms += current_arms;
                measured_data.power_w += power_w;
                measured_data.freq_hz += freq_hz;
                measurement_counter += 1;
                let upload_interval_seconds = MEASUREMENT_CONFIG.lock().ok().map(|guard| guard.borrow().as_ref().unwrap().upload_interval_seconds);
                if let Some(upload_interval_seconds) = upload_interval_seconds {
                    if  measurement_counter >= upload_interval_seconds {
                        measured_data.current_arms /= measurement_counter as f32;
                        measured_data.power_w /= measurement_counter as f32;
                        measured_data.freq_hz /= measurement_counter as f32;
                        measured_data_queue.send_back(measured_data, TickType::new(NON_BLOCK).ticks()).ok();
                        measurement_counter = 0;
                        measured_data = Default::default();
                    }
                }
            }
        }
    }
}
