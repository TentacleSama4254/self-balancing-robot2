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
use esp_hal::i2c::master::I2c;
use esp_hal::timer::timg::TimerGroup;
use esp_println as _;
use self_balancing_robot2::imu::{FreeSixIMU, SensorStatus};
use self_balancing_robot2::i2c_wrapper::I2cWrapper;
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
    
    // Initialize IMU with delay function and advanced filtering
    match imu.init_with_advanced_filtering(&mut delay_fn) {
        Ok(_) => info!("IMU initialized successfully with advanced filtering enabled!"),
        Err(_) => {
            info!("Failed to initialize IMU with advanced filtering, falling back to standard mode!");
            let _ = imu.init(&mut delay_fn);
        }
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

    // Variables for auto-calibration
    let mut last_auto_cal_time = 0u64;
    let mut stability_counter = 0u16;
    const AUTO_CAL_INTERVAL_MICROS: u64 = 5_000_000; // 5 seconds
    const STABLE_COUNT_THRESHOLD: u16 = 10;          // Number of stable readings required
    
    loop {
        // Use formatted values for better readability (3 decimal places)
        match imu.get_formatted_values() {
            Ok((int_parts, frac_parts)) => {
                // Get sensor health status
                let sensor_status = imu.get_sensor_health();
                
                match sensor_status.status {
                    SensorStatus::Ok => {
                        info!(
                            "Accel: X={}.{:03} Y={}.{:03} Z={}.{:03} g, Gyro: X={}.{:03} Y={}.{:03} Z={}.{:03} deg/s",
                            int_parts[0], frac_parts[0], int_parts[1], frac_parts[1], int_parts[2], frac_parts[2], 
                            int_parts[3], frac_parts[3], int_parts[4], frac_parts[4], int_parts[5], frac_parts[5]
                        );
                        
                        // Count consecutive stable readings for auto-calibration
                        if int_parts[3].abs() <= 0 && int_parts[4].abs() <= 0 && int_parts[5].abs() <= 0 && 
                           frac_parts[3] < 100 && frac_parts[4] < 100 && frac_parts[5] < 100 {
                            stability_counter += 1;
                        } else {
                            stability_counter = 0;
                        }
                        
                        // Auto-calibration when device is stable
                        if stability_counter >= STABLE_COUNT_THRESHOLD && 
                           micros - last_auto_cal_time > AUTO_CAL_INTERVAL_MICROS {
                            
                            info!("Device stable - performing auto-calibration");
                            
                            // Set status to calibrating
                            match imu.auto_calibrate(20, &mut delay_fn) {
                                Ok(true) => {
                                    info!("Auto-calibration successful");
                                    last_auto_cal_time = micros;
                                    
                                    // Reset filter state after calibration to avoid sudden jumps
                                    imu.reset_filter_state();
                                    info!("Filter state reset after calibration");
                                },
                                Ok(false) => {
                                    info!("Auto-calibration skipped - device not stable enough");
                                    stability_counter = 0;
                                },
                                Err(_) => {
                                    info!("Auto-calibration failed");
                                }
                            }
                        }
                    },
                    SensorStatus::Disconnected => {
                        info!("⚠️ IMU DISCONNECTED! Reconnect the sensor and reset.");
                        stability_counter = 0;
                    },
                    SensorStatus::Faulty => {
                        info!("⚠️ IMU FAULTY READINGS DETECTED! Check sensor connections.");
                        stability_counter = 0;
                    },
                    SensorStatus::NeedsCalibration => {
                        info!("IMU needs calibration after reconnection, performing calibration...");
                        // Perform a brief calibration after reconnection
                        match imu.calibrate(5000, 100, &mut delay_fn) {
                            Ok(_) => {
                                info!("Post-reconnection calibration successful");
                                imu.reset_filter_state(); // Reset filters after calibration
                            },
                            Err(_) => info!("Post-reconnection calibration failed")
                        };
                    },
                    SensorStatus::ExcessiveDrift => {
                        info!("⚠️ IMU EXCESSIVE DRIFT DETECTED! Will attempt auto-calibration...");
                        
                        // Only attempt auto-calibration if it's been a while since the last one
                        if micros - last_auto_cal_time > AUTO_CAL_INTERVAL_MICROS / 2 {
                            // Try to correct the drift with auto-calibration
                            match imu.auto_calibrate(10, &mut delay_fn) {
                                Ok(true) => {
                                    info!("Auto-calibration for drift correction successful");
                                    last_auto_cal_time = micros;
                                    
                                    // Reset filter state after calibration
                                    imu.reset_filter_state();
                                },
                                _ => {
                                    info!("Unable to auto-calibrate - device may be moving");
                                    stability_counter = 0;
                                }
                            }
                        }
                    },
                    SensorStatus::Calibrating => {
                        info!("IMU is currently calibrating... Please keep the device still.");
                        stability_counter = 0; // Reset stability counter during calibration
                    }
                }
                
                // Try to read orientation with formatted values if the sensor is working
                if sensor_status.status != SensorStatus::Disconnected && 
                   sensor_status.status != SensorStatus::Faulty {
                    
                    match imu.get_formatted_euler_angles(micros) {
                        Ok((angle_int, angle_frac)) => {
                            info!("Roll={}.{:03}, Pitch={}.{:03}, Yaw={}.{:03} degrees", 
                                 angle_int[0], angle_frac[0], angle_int[1], angle_frac[1], angle_int[2], angle_frac[2]);
                        },
                        Err(_) => {
                            info!("Failed to read euler angles");
                        }
                    }
                }
                
                // Display sensor health stats periodically
                if micros % 1_000_000 == 0 {  // Every second
                    info!("Sensor health: Disconnects={}, Faulty readings={}, Status={}",
                         sensor_status.disconnect_count, sensor_status.faulty_reading_count, sensor_status.status);
                    
                    // Report gyro bias values for monitoring drift correction
                    let gyro_bias = imu.get_gyro_bias();
                    info!("Gyro bias: X={}.{:03} Y={}.{:03} Z={}.{:03} deg/s",
                          (gyro_bias[0] * 1000.0) as i32 / 1000, 
                          ((gyro_bias[0] * 1000.0).abs() as u32) % 1000,
                          (gyro_bias[1] * 1000.0) as i32 / 1000, 
                          ((gyro_bias[1] * 1000.0).abs() as u32) % 1000,
                          (gyro_bias[2] * 1000.0) as i32 / 1000, 
                          ((gyro_bias[2] * 1000.0).abs() as u32) % 1000);
                }
            },
            Err(_) => {
                info!("Failed to read IMU values");
                stability_counter = 0;
            }
        }

        micros += micros_per_loop;
        Timer::after(Duration::from_micros(micros_per_loop)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.1/examples/src/bin
}
