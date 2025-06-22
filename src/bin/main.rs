             #![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::{CpuClock};
use esp_hal::gpio::{Output};
use esp_hal::i2c::master::I2c;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::systimer::SystemTimer;
use esp_hal::peripheral::Peripheral; // Needed for peripheral take()
use esp_println as _;
use self_balancing_robot2::imu::FreeSixIMU;
use self_balancing_robot2::i2c_wrapper::I2cWrapper;
use self_balancing_robot2::motor::stepper::StepperMotor;
use core::cell::RefCell;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

const CALIBRATE_IMU: bool = false;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.4.0
    info!("Starting self-balancing robot with IMU...");

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    
    // Configure I2C pins 
    let io25scl = peripherals.GPIO25;
    let io33sda = peripherals.GPIO33;

    // Configure TMC2209 stepper driver pins
    // DIR - pin 18, STEP - pin 19
    let dir_pin = peripherals.GPIO18.into_push_pull_output();
    let step_pin = peripherals.GPIO19.into_push_pull_output();
    
    // Initialize TMC2209 stepper motor driver
    let mut stepper_motor = StepperMotor::new_esp32(dir_pin, step_pin);
    stepper_motor.set_acceleration(1000.0); // steps/s²
    stepper_motor.set_speed(200.0);         // steps/s (starting speed)
    info!("Stepper motor initialized!");
    
    // Initialize I2C
    let i2c = I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default(),
    )
    .unwrap()
    .with_sda(io33sda)
    .with_scl(io25scl);
    
    // Wrap the I2C in a RefCell so it can be shared
    let i2c_cell = RefCell::new(i2c);
    let i2c_wrapper = I2cWrapper::new(&i2c_cell);

    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    // Initialize system timer for precise timing
    let system_timer = SystemTimer::new(peripherals.SYSTIMER);
    
    info!("Embassy initialized!");

    // Initialize IMU with our cloneable wrapper
    let mut imu = FreeSixIMU::new(i2c_wrapper);
    
    // Define a delay function we'll use for both initialization and calibration
    let mut delay_fn = |ms| {
        // Basic delay implementation - in real code, use a proper delay function
        let cycles_per_ms = 240_000; // Assuming ESP32 at ~240MHz
        for _ in 0..ms * cycles_per_ms {
            core::hint::spin_loop();
        }
    };
    
    // Initialize IMU with delay function
    match imu.init(&mut delay_fn) {
        Ok(_) => info!("IMU initialized successfully!"),
        Err(_) => info!("Failed to initialize IMU!"),
    };
    
    // Only perform calibration if CALIBRATE_IMU is true
    if CALIBRATE_IMU {
        info!("Starting IMU calibration. Please keep the device still and level for ~10 seconds...");
        // Increase samples for more stable calibration (12,000 samples for gyro, 200 for accelerometer)
        match imu.calibrate(12000, 200, &mut delay_fn) {
            Ok(_) => info!("IMU calibrated successfully!"),
            Err(_) => {
                // In a real implementation, you might want to use a more specific error handling
                info!("Failed to calibrate IMU! Device may have unstable readings.");
                // Attempt a basic calibration as fallback
                let _ = imu.zero_calibrate(1000, &mut delay_fn);
            },
        };
        
        // Small delay after calibration
        delay_fn(500);
    } else {
        info!("Skipping IMU calibration as per configuration.");
    }

    // Read initial values to verify calibration
    info!("Reading initial calibrated values:");
    match imu.get_formatted_values() {
        Ok((int_parts, frac_parts)) => {
            info!(
                "Initial Accel: X={}.{:03} Y={}.{:03} Z={}.{:03} g, Gyro: X={}.{:03} Y={}.{:03} Z={}.{:03} deg/s",
                int_parts[0], frac_parts[0], int_parts[1], frac_parts[1], int_parts[2], frac_parts[2], 
                int_parts[3], frac_parts[3], int_parts[4], frac_parts[4], int_parts[5], frac_parts[5]
            );
            
            // We still need the raw values to check calibration quality
            let values = imu.get_values().unwrap();
            
            // Good calibration should have accel Z around 1.0g and other values close to zero,
            // and gyro values all close to zero when not moving
            if values[0].abs() < 0.1 && values[1].abs() < 0.1 && (values[2] - 1.0).abs() < 0.1 &&
               values[3].abs() < 0.1 && values[4].abs() < 0.1 && values[5].abs() < 0.1 {
                info!("Calibration looks excellent!");
            } else if values[0].abs() < 0.2 && values[1].abs() < 0.2 && (values[2] - 1.0).abs() < 0.2 &&
                     values[3].abs() < 0.3 && values[4].abs() < 0.3 && values[5].abs() < 0.3 {
                info!("Calibration looks acceptable.");
            } else {
                info!("Calibration is not optimal. Please keep device perfectly still and level during calibration.");
                
                // Provide more specific feedback on which sensors need attention
                if values[0].abs() > 0.2 || values[1].abs() > 0.2 || (values[2] - 1.0).abs() > 0.2 {
                    info!("Accelerometer values are off. Device may not be perfectly level.");
                }
                if values[3].abs() > 0.3 || values[4].abs() > 0.3 || values[5].abs() > 0.3 {
                    info!("Gyroscope shows drift. Device may be moving slightly during calibration.");
                }
            }
        },
        Err(_) => {
            info!("Failed to read initial values!");
        }
    }

    // TODO: Spawn some tasks
    let _ = spawner;

    let mut micros = 0u64;
    let micros_per_loop = 10_000; // 10ms per loop
    
    // Variables for motor control
    let mut current_roll = 0.0;
    let mut motor_step_count = 0;
    let motor_update_interval = 5; // Update motor every 5 loop iterations

    loop {
        // Get current time in microseconds for precise motor timing
        let current_time_micros = system_timer.now().ticks() as u64;
        
        // Use formatted values for better readability (3 decimal places)
        match imu.get_formatted_values() {
            Ok((int_parts, frac_parts)) => {
                // Only print IMU values occasionally to reduce console spam
                if micros % 100_000 == 0 {
                    info!(
                        "Accel: X={}.{:03} Y={}.{:03} Z={}.{:03} g, Gyro: X={}.{:03} Y={}.{:03} Z={}.{:03} deg/s",
                        int_parts[0], frac_parts[0], int_parts[1], frac_parts[1], int_parts[2], frac_parts[2], 
                        int_parts[3], frac_parts[3], int_parts[4], frac_parts[4], int_parts[5], frac_parts[5]
                    );
                }
                
                // Try to read orientation with formatted values
                match imu.get_euler_angles(current_time_micros) {
                    Ok(angles) => {
                        let roll = angles[0];
                        let pitch = angles[1];
                        
                        // Update current roll for motor control
                        current_roll = roll;
                        
                        if micros % 100_000 == 0 {
                            info!(
                                "Roll={} Pitch={} Yaw={} degrees, Motor pos: {}",
                                roll as i32, pitch as i32, angles[2] as i32, stepper_motor.get_position()
                            );
                        }
                    },
                    Err(_) => {
                        info!("Failed to read euler angles");
                    }
                }
            },
            Err(_) => {
                info!("Failed to read IMU values");
            }
        }

        // Control stepper motor based on roll angle
        // We use the roll angle as input for the balance controller
        
        // Simple proportional control for testing
        // Only update motor every few iterations for better performance
        motor_step_count += 1;
        if motor_step_count >= motor_update_interval {
            motor_step_count = 0;
            
            // Threshold to avoid tiny movements
            if current_roll.abs() > 1.0 {
                // Set motor speed proportional to the roll angle
                let motor_speed = current_roll * 10.0; // 10 steps/s per degree
                stepper_motor.set_speed(motor_speed);
                
                // Determine step direction based on roll angle
                // Negative roll → positive steps, positive roll → negative steps
                // This creates a counterbalancing effect
                let step_direction = if current_roll > 0.0 { -1 } else { 1 };
                
                // Execute the step
                match stepper_motor.move_steps(step_direction, current_time_micros) {
                    Ok(stepped) => {
                        if stepped && micros % 100_000 == 0 {
                            info!("Motor step taken: direction={}, roll={}", step_direction, current_roll);
                        }
                    },
                    Err(_) => {
                        if micros % 1_000_000 == 0 {
                            info!("Motor step error");
                        }
                    }
                }
            } else if micros % 1_000_000 == 0 {
                // If roll is small, stop the motor
                stepper_motor.set_speed(0.0);
                info!("Motor idle - roll angle too small");
            }
        }

        micros += micros_per_loop;
        Timer::after(Duration::from_micros(micros_per_loop)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.1/examples/src/bin
}
