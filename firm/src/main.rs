use std::time::Duration;

use esp_idf_svc::hal::{peripherals::Peripherals, adc::{AdcDriver, attenuation, AdcChannelDriver, config::Config}, timer, delay::NON_BLOCK};
use esp_idf_svc::hal::delay::TickType;
use esp_idf_svc::hal::task::queue::Queue;

fn main() -> anyhow::Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;

    // Initialize ADC
    let mut adc = AdcDriver::new(peripherals.adc1, &Config::new().calibration(true))?;
    let mut adc_pin: AdcChannelDriver<{ attenuation::DB_6}, _> = AdcChannelDriver::new(peripherals.pins.gpio8)?;
    const SAMPLES_PER_SECOND: usize = 6000;
    const OFFSET_MV: i16 = 1100;

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
