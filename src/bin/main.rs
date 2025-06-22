#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Io, Level, Output, OutputConfig};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::time::Instant;
use esp_println as _;
use self_balancing_robot2::motor::stepper::{OutputWrapper, StepperMotor};

// Constants for stepper motor rotation
const STEPS_PER_REVOLUTION: i32 = 200 * 32; // 200 steps * 32 microsteps
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
    let io = Io::new(peripherals.IO_MUX);

    // Initialize pins as outputs
    let mut dir_pin = Output::new(peripherals.GPIO18, Level::Low, OutputConfig::default());
    let mut step_pin = Output::new(peripherals.GPIO19, Level::Low, OutputConfig::default());
    
    // Create motor driver
    let dir_pin_wrapped = OutputWrapper::new(&mut dir_pin);
    let step_pin_wrapped = OutputWrapper::new(&mut step_pin);
    let mut stepper_motor = StepperMotor::new_esp32(dir_pin_wrapped, step_pin_wrapped);
    
    // Initialize timer for embassy
    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);
    
    info!("Stepper motor initialized!");
    info!("Steps per revolution: {}", STEPS_PER_REVOLUTION);
    info!("Total steps for 4 rotations: {}", TOTAL_STEPS);

    // Motor state variables
    let mut current_direction = true; // true = forward, false = reverse
    let mut target_position = 0i32;
    let mut phase = 0; // 0 = accelerate, 1 = constant speed, 2 = decelerate
    let mut cycle_count = 0;
    
    // Speed and acceleration parameters
    let max_speed = 1200.0; // Maximum speed in steps/second
    let acceleration = 800.0; // Acceleration in steps/second²
    let mut current_speed = 0.0;
    
    // Set initial target position (4 full rotations forward)
    target_position = TOTAL_STEPS;
    
    stepper_motor.set_acceleration(acceleration);
    stepper_motor.set_speed(0.0);
    
    info!("Starting rotation cycle...");
    
    // Main motor control loop
    loop {
        let current_time = Instant::now().duration_since_epoch().as_micros() as u64;
        let current_position = stepper_motor.get_position();
        let distance_to_target = (target_position - current_position).abs();
        
        // Calculate speeds for acceleration/deceleration profile
        let accel_distance = (max_speed * max_speed) / (2.0 * acceleration); // Distance needed to reach max speed
        let decel_distance = accel_distance as i32; // Same distance to decelerate
        
        // Determine which phase we're in
        if distance_to_target > decel_distance {
            if current_speed < max_speed {
                // Acceleration phase
                phase = 0;
                current_speed = (current_speed + acceleration * 0.01).min(max_speed); // Increment speed
            } else {
                // Constant speed phase
                phase = 1;
                current_speed = max_speed;
            }
        } else {
            // Deceleration phase
            phase = 2;
            let decel_ratio = distance_to_target as f32 / decel_distance as f32;
            current_speed = (max_speed * decel_ratio).max(50.0); // Minimum speed to prevent stalling
        }
        
        // Set direction and speed
        if current_direction {
            stepper_motor.set_speed(current_speed);
        } else {
            stepper_motor.set_speed(-current_speed);
        }
        
        // Execute motor step
        match stepper_motor.move_continuous(current_time) {
            Ok(stepped) => {
                // Log progress occasionally
                if stepped && current_position % 1000 == 0 {
                    info!(
                        "Cycle: {}, Phase: {}, Pos: {}/{}, Speed: {}",
                        cycle_count,
                        match phase {
                            0 => "ACCEL",
                            1 => "CONST",
                            2 => "DECEL",
                            _ => "UNKNOWN"
                        },
                        current_position,
                        target_position,
                        current_speed as i32
                    );
                }
            },
            Err(_) => {
                info!("Motor step error");
            }
        }
        
        // Check if we've reached the target position
        if distance_to_target <= 2 { // Small tolerance for position accuracy
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
                // Going forward: target is current position + 4 rotations
                target_position = current_position + TOTAL_STEPS;
                info!("Starting FORWARD rotation #{}", (cycle_count + 1) / 2);
            } else {
                // Going reverse: target is current position - 4 rotations  
                target_position = current_position - TOTAL_STEPS;
                info!("Starting REVERSE rotation #{}", cycle_count / 2);
            }
            
            // Reset speed for new cycle
            current_speed = 0.0;
            stepper_motor.set_speed(0.0);
        }
        
        // Small delay to prevent CPU hogging
        Timer::after(Duration::from_millis(1)).await;
    }
}

// The improved implementation uses a single main loop without separate tasks
