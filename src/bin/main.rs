#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Io, Level, Output, OutputConfig};
use esp_hal::timer::timg::TimerGroup;
use esp_println as _;

// Constants for stepper motor rotation
const STEPS_PER_REVOLUTION: i32 = 200 * 8; // 200 steps * 8 microsteps (match stepper.rs)
const FULL_ROTATIONS: i32 = 4;
const TOTAL_STEPS: i32 = STEPS_PER_REVOLUTION * FULL_ROTATIONS;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

// This creates a default app-descriptor required by the esp-idf bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    info!("Starting stepper motor rotation demo - 4 full rotations with acceleration/deceleration");

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    
    // Configure TMC2209 stepper driver pins
    let _io = Io::new(peripherals.IO_MUX);

    // Initialize pins as outputs
    let mut dir_pin = Output::new(peripherals.GPIO18, Level::Low, OutputConfig::default());
    let mut step_pin = Output::new(peripherals.GPIO19, Level::Low, OutputConfig::default());
    
    // Initialize timer for embassy
    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);
    
    info!("Stepper motor initialized!");
    info!("Steps per revolution: {}", STEPS_PER_REVOLUTION);
    info!("Total steps for 4 rotations: {}", TOTAL_STEPS);

    // Motor state variables
    let mut current_direction = true; // true = forward, false = reverse
    let mut target_position = TOTAL_STEPS;
    let mut current_position = 0i32;
    let mut cycle_count = 0;
    
    // Speed and acceleration parameters
    let max_speed = 1800.0; // Maximum speed in steps/second (reduced for stability)
    let acceleration = 800.0; // Acceleration in steps/second²
    let mut current_speed = 0.0;
    
    info!("Starting rotation cycle...");
    
    // Set initial direction
    dir_pin.set_high(); // Forward
    Timer::after(Duration::from_micros(10)).await; // Direction setup time
    
    // Main synchronized control loop
    loop {
        let distance_to_target = (target_position - current_position).abs();
        
        // Check if we've reached the target position
        if distance_to_target <= 2 {
            info!(
                "Target reached! Position: {}, Target: {}, Direction: {}",
                current_position,
                target_position,
                if current_direction { "FORWARD" } else { "REVERSE" }
            );
            
            // Pause briefly at the end of each direction
            Timer::after(Duration::from_millis(500)).await;
            
            // Switch direction and set new target
            current_direction = !current_direction;
            cycle_count += 1;
            
            if current_direction {
                target_position = current_position + TOTAL_STEPS;
                info!("Starting FORWARD rotation #{}", (cycle_count + 1) / 2);
                dir_pin.set_high();
            } else {
                target_position = current_position - TOTAL_STEPS;
                info!("Starting REVERSE rotation #{}", cycle_count / 2);
                dir_pin.set_low();
            }
            
            // Direction setup time after change
            Timer::after(Duration::from_micros(10)).await;
            
            // Reset speed for new cycle
            current_speed = 50.0; // Start with minimum speed
            continue;
        }
        
        // Calculate speeds for acceleration/deceleration profile
        let accel_distance = (max_speed * max_speed) / (2.0 * acceleration);
        let decel_distance = accel_distance as i32;
        
        // Determine target speed
        let target_speed = if distance_to_target > decel_distance {
            if current_speed < max_speed {
                // Acceleration phase - smoother acceleration
                (current_speed + acceleration * 0.02_f32).min(max_speed)
            } else {
                // Constant speed phase
                max_speed
            }
        } else {
            // Deceleration phase
            let decel_ratio = distance_to_target as f32 / decel_distance as f32;
            (max_speed * decel_ratio).max(50.0)
        };
        
        current_speed = target_speed;
        
        // Calculate step delay in microseconds
        let step_delay_us = if current_speed > 0.0 {
            ((1_000_000.0 / current_speed) as u64).max(100) // Minimum 100μs
        } else {
            1000 // Default 1ms if speed is 0
        };
        
        // Generate synchronized step pulse
        step_pin.set_high();
        Timer::after(Duration::from_micros(2)).await; // 2μs pulse width
        step_pin.set_low();
        
        // Update position
        if current_direction {
            current_position += 1;
        } else {
            current_position -= 1;
        }
        
        // Log progress occasionally
        if current_position % 1000 == 0 {
            info!(
                "Pos: {}/{}, Speed: {} steps/s, Delay: {}μs",
                current_position,
                target_position,
                current_speed as i32,
                step_delay_us
            );
        }
        
        // Wait for the remaining time to achieve target speed
        let remaining_delay = if step_delay_us > 2 { step_delay_us - 2 } else { 1 };
        Timer::after(Duration::from_micros(remaining_delay)).await;
    }
}
