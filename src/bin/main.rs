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
use esp_hal::gpio::{Output, OutputConfig, Level};
use esp_hal::uart::{self, Uart};
use esp_hal::timer::timg::TimerGroup;
use esp_println as _;
use self_balancing_robot2::imu::FreeSixIMU;
use self_balancing_robot2::i2c_wrapper::I2cWrapper;
use self_balancing_robot2::stepper::{self, Tmc2209Driver};
use core::cell::RefCell;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

const CALIBRATE_IMU: bool = false;
const DRIVER_ADDRESS: u8 = 0b00;
const R_SENSE: f32 = 0.11;

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
    let io22scl = peripherals.GPIO22;
    let io23sda = peripherals.GPIO23;

    
    // Initialize I2C
    let i2c = I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default(),
    )
    .unwrap()
    .with_sda(io23sda)
    .with_scl(io22scl);
    
    // Wrap the I2C in a RefCell so it can be shared
    let i2c_cell = RefCell::new(i2c);
    let i2c_wrapper = I2cWrapper::new(&i2c_cell);

    // Setup UART for the TMC2209 driver
    let uart = Uart::new(
        peripherals.UART1,
        uart::Config::default(),
    )
    .unwrap()
    .with_tx(peripherals.GPIO17)
    .with_rx(peripherals.GPIO16);
    let (_rx, tx) = uart.split();
    let tx = tx.into_async();

    // Configure step and direction pins
    let step_pin = Output::new(peripherals.GPIO18, Level::Low, OutputConfig::default());
    let dir_pin = Output::new(peripherals.GPIO19, Level::Low, OutputConfig::default());

    // Create TMC2209 driver instance
    let driver = Tmc2209Driver::new(tx, DRIVER_ADDRESS, R_SENSE);

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

    // Spawn the stepper task to run concurrently
    spawner.spawn(stepper::run_stepper_task(step_pin, dir_pin, driver)).unwrap();

    let mut micros = 0u64;
    let micros_per_loop = 10_000; // 10ms per loop

    loop {
        // Use formatted values for better readability (3 decimal places)
        match imu.get_formatted_values() {
            Ok((int_parts, frac_parts)) => {
                info!(
                    "Accel: X={}.{:03} Y={}.{:03} Z={}.{:03} g, Gyro: X={}.{:03} Y={}.{:03} Z={}.{:03} deg/s",
                    int_parts[0], frac_parts[0], int_parts[1], frac_parts[1], int_parts[2], frac_parts[2], 
                    int_parts[3], frac_parts[3], int_parts[4], frac_parts[4], int_parts[5], frac_parts[5]
                );
                
                // Try to read orientation with formatted values
                match imu.get_formatted_euler_angles(micros) {
                    Ok((angle_int, angle_frac)) => {
                        info!("Roll={}.{:03}, Pitch={}.{:03}, Yaw={}.{:03} degrees", 
                             angle_int[0], angle_frac[0], angle_int[1], angle_frac[1], angle_int[2], angle_frac[2]);
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

        micros += micros_per_loop;
        Timer::after(Duration::from_micros(micros_per_loop)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.1/examples/src/bin
}
