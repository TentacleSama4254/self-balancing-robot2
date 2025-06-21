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
use esp_hal::i2c;
use esp_hal::timer::timg::TimerGroup;
use esp_println as _;
use self_balancing_robot2::imu::FreeSixIMU;
use self_balancing_robot2::i2c_wrapper::I2cWrapper;
use fugit::RateExtU32;
use core::cell::RefCell;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

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
    
    // Initialize IMU with delay function
    match imu.init(&mut |ms| {
        // Basic delay implementation - in real code, use a proper delay function
        let cycles_per_ms = 240_000; // Assuming ESP32 at ~240MHz
        for _ in 0..ms * cycles_per_ms {
            core::hint::spin_loop();
        }
    }) {
        Ok(_) => info!("IMU initialized successfully!"),
        Err(_) => info!("Failed to initialize IMU!"),
    };

    // TODO: Spawn some tasks
    let _ = spawner;

    let mut micros = 0u64;
    let micros_per_loop = 10_000; // 10ms per loop

    loop {
        match imu.get_values() {
            Ok(values) => {
                info!(
                    "Accel: X={} Y={} Z={} g, Gyro: X={} Y={} Z={} deg/s",
                    values[0], values[1], values[2], values[3], values[4], values[5]
                );
                
                // Try to read orientation
                match imu.get_euler_angles(micros) {
                    Ok(angles) => {
                        info!("Roll={}, Pitch={}, Yaw={} degrees", angles[0], angles[1], angles[2]);
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
