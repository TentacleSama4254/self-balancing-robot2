#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Io, Level, Output, OutputConfig};
use esp_hal::timer::timg::TimerGroup;
use esp_println as _;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    info!("Simple Stepper Test - Smooth Rotation");

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    
    let _io = Io::new(peripherals.IO_MUX);

    // Initialize pins as outputs
    let mut dir_pin = Output::new(peripherals.GPIO18, Level::Low, OutputConfig::default());
    let mut step_pin = Output::new(peripherals.GPIO19, Level::Low, OutputConfig::default());
    
    // Initialize timer for embassy
    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);
    
    info!("Starting smooth rotation test...");
    
    // Set direction forward
    dir_pin.set_high();
    
    let mut step_count = 0;
    let target_steps = 1600; // 1 full rotation (200 * 8 microsteps)
    
    // Test different speeds
    let speeds = [400, 800, 1200, 1600, 2000]; // steps per second
    
    for &speed in &speeds {
        info!("Testing speed: {} steps/sec", speed);
        
        let step_delay_us = 1_000_000 / speed as u64;
        info!("Step delay: {}μs", step_delay_us);
        
        for _ in 0..target_steps {
            // Proper step pulse for TMC2209
            step_pin.set_high();
            Timer::after(Duration::from_micros(2)).await; // 2μs pulse width
            step_pin.set_low();
            Timer::after(Duration::from_micros(1)).await; // 1μs between pulses
            
            step_count += 1;
            
            // Wait for the remaining time to achieve target speed
            let remaining_delay = if step_delay_us > 3 { step_delay_us - 3 } else { 1 };
            Timer::after(Duration::from_micros(remaining_delay)).await;
        }
        
        info!("Completed {} steps at {} steps/sec", target_steps, speed);
        
        // Pause between speed tests
        Timer::after(Duration::from_millis(1000)).await;
    }
    
    info!("Speed test complete. Starting continuous rotation...");
    
    // Continuous smooth rotation
    loop {
        let step_delay_us = 500; // 2000 steps/sec
        
        // Generate step pulse
        step_pin.set_high();
        Timer::after(Duration::from_micros(2)).await;
        step_pin.set_low();
        Timer::after(Duration::from_micros(1)).await;
        
        step_count += 1;
        
        // Log every full rotation
        if step_count % 1600 == 0 {
            info!("Completed rotation, total steps: {}", step_count);
        }
        
        // Remaining delay for target speed
        Timer::after(Duration::from_micros(step_delay_us - 3)).await;
    }
}
