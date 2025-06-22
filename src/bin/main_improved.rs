#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Io, Level, Output, OutputConfig};
use esp_hal::i2c::master::{self, I2c};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::time::Instant;
use esp_println as _;
use self_balancing_robot2::imu::FreeSixIMU;
use self_balancing_robot2::i2c_wrapper::I2cWrapper;
use self_balancing_robot2::motor::stepper::{OutputWrapper, StepperMotor};
use core::cell::RefCell;
use core::sync::atomic::{AtomicI32, Ordering};

// Simple atomic state for communication between components
static ROLL_ANGLE: AtomicI32 = AtomicI32::new(0);
static MOTOR_POSITION: AtomicI32 = AtomicI32::new(0);

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

// This creates a default app-descriptor required by the esp-idf bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    info!("Starting self-balancing robot with IMU - Improved Motor Performance");

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    
    // Configure I2C pins 
    let io25scl = peripherals.GPIO25;
    let io33sda = peripherals.GPIO33;

    // Configure TMC2209 stepper driver pins
    let io = Io::new(peripherals.IO_MUX);

    // Initialize pins as outputs
    let mut dir_pin = Output::new(peripherals.GPIO18, Level::Low, OutputConfig::default());
    let mut step_pin = Output::new(peripherals.GPIO19, Level::Low, OutputConfig::default());
    
    // Create motor driver with enhanced settings
    let dir_pin_wrapped = OutputWrapper::new(&mut dir_pin);
    let step_pin_wrapped = OutputWrapper::new(&mut step_pin);
    let mut stepper_motor = StepperMotor::new_esp32(dir_pin_wrapped, step_pin_wrapped);
    
    // Configure motor for faster movement
    stepper_motor.set_acceleration(3000.0);  // Higher acceleration for responsive movement
    stepper_motor.set_speed(1000.0);         // Higher initial speed
    
    info!("Stepper motor initialized with high-performance settings!");
    
    // Initialize I2C
    let i2c = I2c::new(
        peripherals.I2C0,
        master::Config::default(),
    )
    .unwrap()
    .with_sda(io33sda)
    .with_scl(io25scl);
    
    // Create I2C wrapper
    let i2c_cell = RefCell::new(i2c);
    let i2c_wrapper = I2cWrapper::new(&i2c_cell);
    
    // Initialize IMU
    let mut imu = FreeSixIMU::new(i2c_wrapper);
    
    // Initialize timer for embassy
    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);
    
    info!("Embassy initialized!");

    // Basic delay function
    let mut delay_fn = |ms| {
        // Assuming ESP32 at ~240MHz
        let cycles_per_ms = 240_000;
        for _ in 0..ms * cycles_per_ms {
            core::hint::spin_loop();
        }
    };
    
    // Initialize IMU
    match imu.init(&mut delay_fn) {
        Ok(_) => info!("IMU initialized successfully!"),
        Err(_) => info!("Failed to initialize IMU!"),
    };

    // Skip calibration for faster startup in testing
    info!("Skipping IMU calibration for testing purposes.");

    // Loop counters
    let mut imu_counter = 0;
    let mut motor_counter = 0;
    
    // Main loop with non-blocking execution
    loop {
        let current_time = Instant::now().duration_since_epoch().as_micros() as u64;
        
        // Process IMU data (every 10ms)
        if imu_counter % 10 == 0 {
            match imu.get_euler_angles(current_time) {
                Ok(angles) => {
                    let roll = angles[0];
                    
                    // Store roll value for inter-task communication
                    ROLL_ANGLE.store((roll * 100.0) as i32, Ordering::Relaxed);
                    
                    // Log IMU readings occasionally
                    if imu_counter % 100 == 0 {
                        info!(
                            "Roll={}, Pitch={}, Yaw={} degrees", 
                            roll as i32, angles[1] as i32, angles[2] as i32
                        );
                    }
                },
                Err(_) => {
                    if imu_counter % 100 == 0 {
                        info!("IMU read error");
                    }
                }
            }
        }
        
        // Process motor control (every 5ms for faster response)
        if motor_counter % 5 == 0 {
            // Get roll from shared atomic
            let roll_int = ROLL_ANGLE.load(Ordering::Relaxed);
            let roll = (roll_int as f32) / 100.0;
            
            // Much more aggressive motor control for testing
            if roll.abs() > 0.3 {  // More sensitive threshold (was 1.0)
                // Higher gain for more dramatic movement
                let motor_speed = roll * 120.0;  // Increased from 50.0 to 120.0
                stepper_motor.set_speed(motor_speed.abs().min(1500.0));  // Higher speed cap
                
                // Determine direction based on roll
                let step_direction = if roll > 0.0 { -1 } else { 1 };
                
                // More steps per cycle for dramatic movement
                let steps_to_take = (roll.abs() * 8.0).ceil() as i32;  // Increased from 5.0 to 8.0
                let step_direction = step_direction * steps_to_take.max(1);
                
                // Execute motor step
                match stepper_motor.move_steps(step_direction, current_time) {
                    Ok(stepped) => {
                        if stepped && motor_counter % 100 == 0 {
                            let abs_steps = if step_direction < 0 { -step_direction } else { step_direction };
                            info!(
                                "Motor steps: dir={}, steps={}, roll={}, current_pos={}",
                                if step_direction > 0 { 1 } else { -1 },
                                abs_steps,
                                roll as i32,
                                stepper_motor.get_position()
                            );
                        }
                    },
                    Err(_) => {
                        if motor_counter % 100 == 0 {
                            info!("Motor step error");
                        }
                    }
                }
                
                // Store motor position in shared atomic
                MOTOR_POSITION.store(stepper_motor.get_position(), Ordering::Relaxed);
            } else {
                if motor_counter % 100 == 0 {
                    stepper_motor.set_speed(0.0);
                    info!("Motor idle - roll angle too small");
                }
            }
        }
        
        // Increment counters
        imu_counter += 1;
        motor_counter += 1;
        
        // Yield to other tasks - short delay to prevent CPU hogging
        Timer::after(Duration::from_millis(1)).await;
    }
}
