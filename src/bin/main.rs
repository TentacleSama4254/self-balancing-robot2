#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use esp_println::println;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::{CpuClock};
use esp_hal::i2c::master::I2c;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_println as _;
use self_balancing_robot2::imu::FreeSixIMU;
use self_balancing_robot2::i2c_wrapper::I2cWrapper;
use core::cell::RefCell;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

const CALIBRATE_IMU: bool = false;

// Stepper motor task
#[embassy_executor::task]
async fn stepper_motor_task(mut step_pin: Output<'static>, mut dir_pin: Output<'static>) {
    println!("Starting stepper motor control task");
    
    let mut step_count = 0u32;
    let steps_per_rotation = 200; // Standard stepper motor (1.8° per step)
    let rotations_before_direction_change = 3; // Change direction every 3 rotations
    let step_delay_ms = 5; // 5ms between steps (200 steps/s)
    
    // Start with clockwise direction
    dir_pin.set_high();
    let mut clockwise = true;
    
    loop {
        // Check if we need to change direction
        if step_count >= (rotations_before_direction_change * steps_per_rotation) {
            clockwise = !clockwise;
            if clockwise {
                dir_pin.set_high();
                println!("Stepper direction: Clockwise");
            } else {
                dir_pin.set_low();
                println!("Stepper direction: Counter-clockwise");
            }
            step_count = 0; // Reset step counter
        }
        
        // Generate step pulse
        step_pin.set_high();
        Timer::after(Duration::from_micros(10)).await; // Short pulse
        step_pin.set_low();
        
        step_count += 1;
        
        // Log progress every full rotation
        if step_count % steps_per_rotation == 0 {
            let rotation_num = step_count / steps_per_rotation;
            if clockwise {
                println!("Completed rotation {} (CW)", rotation_num);
            } else {
                println!("Completed rotation {} (CCW)", rotation_num);
            }
        }
        
        // Wait before next step
        Timer::after(Duration::from_millis(step_delay_ms)).await;
    }
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.4.0
    println!("Starting self-balancing robot with IMU...");
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);    // Configure GPIO pins for stepper motor
    let step_pin = Output::new(peripherals.GPIO18, Level::Low, OutputConfig::default());
    let dir_pin = Output::new(peripherals.GPIO19, Level::Low, OutputConfig::default());

      // Configure I2C pins 
    let io22scl = peripherals.GPIO22;
    let io21sda = peripherals.GPIO21;

    
    // Initialize I2C
    let i2c = I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default(),
    )
    .unwrap()
    .with_sda(io21sda)
    .with_scl(io22scl);
    
    // Wrap the I2C in a RefCell so it can be shared
    let i2c_cell = RefCell::new(i2c);
    let i2c_wrapper = I2cWrapper::new(&i2c_cell);

    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);
    println!("Embassy initialized!");
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
        Ok(_) => println!("IMU initialized successfully!"),
        Err(_) => println!("Failed to initialize IMU!"),
        Err(_) => info!("Failed to initialize IMU!"),
    };
    
    // Only perform calibration if CALIBRATE_IMU is true
        println!("Starting IMU calibration. Please keep the device still and level for ~10 seconds...");
        info!("Starting IMU calibration. Please keep the device still and level for ~10 seconds...");
        // Increase samples for more stable calibration (12,000 samples for gyro, 200 for accelerometer)
            Ok(_) => println!("IMU calibrated successfully!"),
            Err(_) => {
                // In a real implementation, you might want to use a more specific error handling
                println!("Failed to calibrate IMU! Device may have unstable readings.");
                // Attempt a basic calibration as fallback
                let _ = imu.zero_calibrate(1000, &mut delay_fn);
            },
            },
        };
        
        // Small delay after calibration
        delay_fn(500);
        println!("Skipping IMU calibration as per configuration.");
        info!("Skipping IMU calibration as per configuration.");
    }

    println!("Reading initial calibrated values:");
    info!("Reading initial calibrated values:");
    match imu.get_formatted_values() {
            println!(
                "Initial Accel: X={}.{:03} Y={}.{:03} Z={}.{:03} g, Gyro: X={}.{:03} Y={}.{:03} Z={}.{:03} deg/s",
                int_parts[0], frac_parts[0], int_parts[1], frac_parts[1], int_parts[2], frac_parts[2], 
                int_parts[3], frac_parts[3], int_parts[4], frac_parts[4], int_parts[5], frac_parts[5]
            );
            );
            
            // We still need the raw values to check calibration quality
            let values = imu.get_values().unwrap();
            
            // Good calibration should have accel Z around 1.0g and other values close to zero,
            // and gyro values all close to zero when not moving
            if values[0].abs() < 0.1 && values[1].abs() < 0.1 && (values[2] - 1.0).abs() < 0.1 &&
                println!("Calibration looks excellent!");
                info!("Calibration looks excellent!");
            } else if values[0].abs() < 0.2 && values[1].abs() < 0.2 && (values[2] - 1.0).abs() < 0.2 &&
                println!("Calibration looks acceptable.");
                info!("Calibration looks acceptable.");
                println!("Calibration is not optimal. Please keep device perfectly still and level during calibration.");
                info!("Calibration is not optimal. Please keep device perfectly still and level during calibration.");
                
                // Provide more specific feedback on which sensors need attention
                    println!("Accelerometer values are off. Device may not be perfectly level.");
                    info!("Accelerometer values are off. Device may not be perfectly level.");
                }
                    println!("Gyroscope shows drift. Device may be moving slightly during calibration.");
                    info!("Gyroscope shows drift. Device may be moving slightly during calibration.");
                }
            }
        },
            println!("Failed to read initial values!");
            info!("Failed to read initial values!");
        }
    }    // TODO: Spawn some tasks
    spawner.spawn(stepper_motor_task(step_pin, dir_pin)).unwrap();    let mut micros = 0u64;
    let micros_per_loop = 100_000; // 100ms per loop (reduced frequency for less spam)
    let mut loop_count = 0u32;

    loop {
        loop_count += 1;
        
        // Only log every 10th iteration (1 second intervals)
        if loop_count % 10 == 0 {
            // Use formatted values for better readability (3 decimal places)
            match imu.get_formatted_values() {
                    println!(
                        "Accel: X={}.{:03} Y={}.{:03} Z={}.{:03} g, Gyro: X={}.{:03} Y={}.{:03} Z={}.{:03} deg/s",
                        int_parts[0], frac_parts[0], int_parts[1], frac_parts[1], int_parts[2], frac_parts[2], 
                        int_parts[3], frac_parts[3], int_parts[4], frac_parts[4], int_parts[5], frac_parts[5]
                    );
                    );
                    
                    // Try to read orientation with formatted values
                    match imu.get_formatted_euler_angles(micros) {
                            println!("Roll={}.{:03}, Pitch={}.{:03}, Yaw={}.{:03} degrees", 
                                 angle_int[0], angle_frac[0], angle_int[1], angle_frac[1], angle_int[2], angle_frac[2]);
                                 angle_int[0], angle_frac[0], angle_int[1], angle_frac[1], angle_int[2], angle_frac[2]);
                        },
                            println!("Failed to read euler angles");
                            info!("Failed to read euler angles");
                        }
                    }
                },
                    println!("Failed to read IMU values");
                    info!("Failed to read IMU values");
                }
            }
        }

        micros += micros_per_loop;
        Timer::after(Duration::from_micros(micros_per_loop)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.1/examples/src/bin
}
