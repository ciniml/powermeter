use std::{time::Duration, str::FromStr, net::Ipv4Addr};

use embedded_svc::{wifi::{AuthMethod, ClientConfiguration, Configuration, AccessPointConfiguration, Protocol, Wifi}, ipv4};

use esp_idf_svc::{hal::{peripherals::Peripherals, adc::{AdcDriver, attenuation, AdcChannelDriver, config::Config}, timer, delay::{NON_BLOCK, self, BLOCK}, gpio::{Pull, PinDriver}}, eventloop::EspSystemEventLoop, wifi::{BlockingWifi, EspWifi, WifiDeviceId, WifiDriver, WifiEvent}, netif::{EspNetif, NetifStack, NetifConfiguration, IpEvent}};
use esp_idf_svc::hal::delay::TickType;
use esp_idf_svc::hal::task::queue::Queue;
use esp_idf_svc::nvs::*;

use log::{info, debug};

static mut ESPTOUCH_KEY: [u8; 16] = [0; 16];

#[derive(Debug, Clone, Copy)]
struct SmartConfigResult([u8; 32], [u8; 64], Option<[u8; 6]>);

static mut SMART_CONFIG_QUEUE: Option<Queue<SmartConfigResult>> = None;

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

    let nvs_default_partition: EspNvsPartition<NvsDefault> = EspDefaultNvsPartition::take()?;
    let mut wifi_nvs = match EspNvs::new(nvs_default_partition.clone(), "wifi", true) {
        Ok(wifi_nvs) => {
            info!("Got namespace {:?} from default partition", "wifi");
            wifi_nvs
        }
        Err(e) => panic!("Could't get namespace {:?}", e),
    };

    let sys_loop = EspSystemEventLoop::take()?;
    let wifi = WifiDriver::new(peripherals.modem, sys_loop.clone(), Some(nvs_default_partition.clone()))?;
    let wifi_ap_mac = wifi.get_mac(WifiDeviceId::Ap)?;
    
    // Conifgure SmartConfig event handler.
    unsafe {
        esp_idf_svc::sys::esp_event_handler_register(
            esp_idf_svc::sys::SC_EVENT,
            esp_idf_svc::sys::ESP_EVENT_ANY_ID,
            Some(sc_event_handler),
            core::ptr::null_mut(),
        );
    }

    // Initialize WiFi STA.
    let wifi = EspWifi::wrap(wifi)?;
    let mut wifi = BlockingWifi::wrap(
        wifi,
        sys_loop.clone(),
    )?;
    wifi.start()?;

    if button.is_low() {
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

        unsafe {
            SMART_CONFIG_QUEUE = Some(Queue::new(1));
            // copy "powermeter" into ESPTOUCH_KEY
            ESPTOUCH_KEY[0..11].copy_from_slice(b"powermeter\0");
        }
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
        info!("Got WiFi configuration: {:?}", wifi_configuration);
    }
    info!("Connecting to WiFi");
    wifi.connect().unwrap();
    info!("Connected to WiFi");
    wifi.wait_netif_up().unwrap();

    // Initialize Timer
    let timer_config = timer::config::Config::new().auto_reload(true).divider(80);
    let mut timer = timer::TimerDriver::new(peripherals.timer00, &timer_config)?;
    timer.set_alarm(timer.tick_hz() / (SAMPLES_PER_SECOND as u64))?;

    // Initialize Queue
    static mut ADC_QUEUE: Option<Queue<i16>> = None;
    unsafe {
        ADC_QUEUE = Some(Queue::new(SAMPLES_PER_SECOND/5));
    }
    unsafe {
        timer.subscribe(move || {
            let value = adc.read(&mut adc_pin).unwrap() as i16;
            if let Some(queue) = &ADC_QUEUE {
                queue.send_back(value, TickType::new(NON_BLOCK).ticks()).ok();
            }
        })?;
    }

    timer.enable_interrupt()?;
    timer.enable_alarm(true)?;
    timer.enable(true)?;

    log::info!("Hello, world!");

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
            offset_average_accumulator += offset_centered_mv;
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
                log::info!("offset: {}, RMS: {}, current: {}, power: {}, freq: {}", offset_mv, rms_v, current_arms, power_w, average_zero_cross_interval.unwrap_or_default());
                
                number_of_samples = 0;
                accumulated_squared = 0;
                zero_cross_accumulator = 0;
                zero_cross_count = 0;
            }
        }
    }
}
