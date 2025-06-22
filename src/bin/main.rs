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
    
    // Configure motor for maximum responsiveness
    stepper_motor.set_acceleration(4000.0); // Ultra-high acceleration for immediate response
    stepper_motor.set_speed(1000.0);        // Higher initial speed for more dramatic movement
    
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
        
        // Process motor control (every 2ms for smooth movement)
        // Using a shorter interval for smoother motion
        if motor_counter % 2 == 0 {
            // Get roll from shared atomic
            let roll_int = ROLL_ANGLE.load(Ordering::Relaxed);
            let roll = (roll_int as f32) / 100.0;
            
            // Create a dead zone to prevent motor jittering when nearly balanced
            let dead_zone = 1.0;  // 1 degree dead zone - adjust based on your sensor noise
            
            if roll.abs() > dead_zone {
                // Only change speed/direction significantly when we exceed the dead zone
                
                // Calculate a smoother motor speed with gradual changes
                // Use a non-linear gain that increases with angle magnitude
                let base_gain = 60.0;  // Lower base gain for smoother motion
                let adaptive_gain = base_gain * (1.0 + roll.abs() * 0.3);
                
                // Calculate motor speed based on current roll angle
                let target_speed = -roll * adaptive_gain; // Negative roll for correct direction
                
                // Apply the speed more gradually to prevent jerky movements
                // Get the current speed and gradually move toward the target
                let current_speed = stepper_motor.get_current_speed();
                let speed_diff = target_speed - current_speed;
                
                // Apply only a fraction of the speed change to smooth transitions
                let speed_change_factor = 0.2;  // 20% change per control cycle
                let new_speed = current_speed + (speed_diff * speed_change_factor);
                
                // Clamp maximum speed for stability
                let clamped_speed = new_speed.abs().min(1500.0);  // Lower top speed for stability
                stepper_motor.set_speed(if new_speed > 0.0 { clamped_speed } else { -clamped_speed });
                
                // Execute motor step - simpler approach with continuous speed control
                match stepper_motor.move_continuous(current_time) {
                    Ok(stepped) => {
                        // Reduce logging frequency to minimize timing interference
                        if stepped && motor_counter % 500 == 0 {
                            info!(
                                "Motor: speed={}, roll={}, pos={}",
                                new_speed as i32,
                                (roll * 10.0) as i32 / 10,
                                stepper_motor.get_position()
                            );
                        }
                    },
                    Err(_) => {
                        if motor_counter % 500 == 0 {
                            info!("Motor step error");
                        }
                    }
                }
                
                // Store motor position in shared atomic
                MOTOR_POSITION.store(stepper_motor.get_position(), Ordering::Relaxed);
            } else {
                // When in the dead zone, gradually stop the motor
                let current_speed = stepper_motor.get_current_speed();
                
                // Only log occasionally to reduce timing disruption
                if motor_counter % 500 == 0 && current_speed != 0.0 {
                    info!("In dead zone - gradually stopping motor");
                }
                
                // Gradually decrease speed to zero instead of immediate stop
                if current_speed.abs() > 50.0 {
                    let new_speed = current_speed * 0.9; // 10% reduction per cycle
                    stepper_motor.set_speed(new_speed);
                    
                    // Still need to step at the reduced speed
                    let _ = stepper_motor.move_continuous(current_time);
                } else if current_speed != 0.0 {
                    // Only zero the speed when we're close to stopped
                    stepper_motor.set_speed(0.0);
                }
            }
        }
        
        // Increment counters
        imu_counter += 1;
        motor_counter += 1;
        
        // Yield to other tasks - short delay to prevent CPU hogging
        // Timer::after(Duration::from_millis(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.1/examples/src/bin
}

// The improved implementation uses a single main loop without separate tasks
