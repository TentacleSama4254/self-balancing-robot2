#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig, OutputPin};
use esp_hal::i2c::master::{self, I2c};
use esp_hal::peripherals::Peripherals;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::time::Instant;
use esp_println as _;
use self_balancing_robot2::imu::FreeSixIMU;
use self_balancing_robot2::i2c_wrapper::I2cWrapper;
use self_balancing_robot2::motor::stepper::{OutputWrapper, StepperMotor};
use core::cell::RefCell;
use core::sync::atomic::{AtomicI32, Ordering};

// Atomic values for communication between tasks
static ROLL_ANGLE: AtomicI32 = AtomicI32::new(0);     // Roll * 100.0
static MOTOR_POSITION: AtomicI32 = AtomicI32::new(0);  // Current motor position
static MOTOR_SPEED: AtomicI32 = AtomicI32::new(0);     // Current motor speed * 10.0
static IMU_UPDATED: AtomicI32 = AtomicI32::new(0);     // Timestamp of last IMU update

// Global storage for peripherals and device handles
static mut PERIPHERALS: Option<Peripherals> = None;
static mut I2C_DEVICE: Option<RefCell<I2c<'static, master::Blocking>>> = None;
static mut DIR_PIN: Option<Output<'static>> = None;
static mut STEP_PIN: Option<Output<'static>> = None;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

// This creates a default app-descriptor required by the esp-idf bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    info!("Starting self-balancing robot with IMU - Embassy Async Tasks");

    // Configure ESP32 at maximum CPU frequency for best performance
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let mut peripherals = esp_hal::init(config);

    // Take TIMG1 out of peripherals using Option::take to avoid partial move
    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);
    info!("Embassy initialized!");

    // Destructure required fields from peripherals before storing globally
    let io25scl = peripherals.GPIO25;
    let io33sda = peripherals.GPIO33;
    let dir_pin = peripherals.GPIO18;
    let step_pin = peripherals.GPIO19;
    let i2c0 = peripherals.I2C0;

    // Store peripherals for global access (now without the moved fields)
    unsafe {
        PERIPHERALS = Some(peripherals);
    }

    // Initialize I2C for IMU
    let blocking_i2c = I2c::new(
        i2c0,
        master::Config::default(),
    )
    .unwrap()
    .with_sda(io33sda)
    .with_scl(io25scl);

    // Store I2C handle
    unsafe {
        I2C_DEVICE = Some(RefCell::new(blocking_i2c));
    }

    // Create I2C wrapper
    let i2c_wrapper = I2cWrapper::new(unsafe { I2C_DEVICE.as_ref().unwrap() });

    // Initialize IMU
    let mut imu = FreeSixIMU::new(i2c_wrapper);

    // Basic delay function for IMU initialization
    let mut delay_fn = |ms| {
        let cycles_per_ms = 240_000; // Assuming ESP32 at ~240MHz
        for _ in 0..ms * cycles_per_ms {
            core::hint::spin_loop();
        }
    };

    // Initialize IMU
    match imu.init(&mut delay_fn) {
        Ok(_) => info!("IMU initialized successfully!"),
        Err(_) => info!("Failed to initialize IMU!"),
    };

    info!("Skipping IMU calibration for testing purposes.");

    // Initialize motor pins
    unsafe {
        DIR_PIN = Some(Output::new(dir_pin, Level::Low, OutputConfig::default()));
        STEP_PIN = Some(Output::new(step_pin, Level::Low, OutputConfig::default()));
    }

    // Create motor driver with wrapped pins
    let dir_pin_wrapped = OutputWrapper::new(unsafe { DIR_PIN.as_mut().unwrap() });
    let step_pin_wrapped = OutputWrapper::new(unsafe { STEP_PIN.as_mut().unwrap() });
    let mut motor = StepperMotor::new_esp32(dir_pin_wrapped, step_pin_wrapped);

    // Configure motor for responsiveness
    motor.set_acceleration(4000.0); // High acceleration for quick response
    motor.set_speed(1000.0);        // Initial speed

    info!("Stepper motor initialized with high-performance settings!");

    // Spawn two tasks
    spawner.spawn(imu_reading_task()).unwrap();
    spawner.spawn(motor_control_task()).unwrap();
    
    // Main task now just monitors the system
    info!("All tasks spawned and running!");
    
    loop {
        // Print status information every 5 seconds
        let roll = (ROLL_ANGLE.load(Ordering::Relaxed) as f32) / 100.0;
        let motor_pos = MOTOR_POSITION.load(Ordering::Relaxed);
        let motor_speed = (MOTOR_SPEED.load(Ordering::Relaxed) as f32) / 10.0;
        
        info!("Status: Roll={} deg, Motor Position={}, Speed={}", 
              roll as i32, motor_pos, motor_speed as i32);
        
        Timer::after(Duration::from_secs(5)).await;
    }
}

// IMU reading task - continuously reads sensor data
#[embassy_executor::task]
async fn imu_reading_task() {
    info!("IMU reading task started");
    
    // Track the reading count for logging
    let mut read_count = 0;
    
    // IMU reading loop - higher frequency (100Hz = 10ms interval)
    loop {
        let current_time = Instant::now().duration_since_epoch().as_micros() as u64;
        
        unsafe {
            if let Some(i2c_ref) = &I2C_DEVICE {
                // Create a new IMU instance inside the loop
                let i2c_wrapper = I2cWrapper::new(i2c_ref);
                let mut imu = FreeSixIMU::new(i2c_wrapper);
                
                match imu.get_euler_angles(current_time) {
                    Ok(angles) => {
                        let roll = angles[0];
                        
                        // Store roll value in atomic for inter-task communication
                        ROLL_ANGLE.store((roll * 100.0) as i32, Ordering::Relaxed);
                        
                        // Set the updated timestamp
                        IMU_UPDATED.store(current_time as i32, Ordering::Relaxed);
                        
                        // Log IMU readings occasionally to reduce serial output overhead
                        read_count += 1;
                        if read_count % 100 == 0 {
                            info!("IMU: Roll={}, Pitch={}, Yaw={} degrees", 
                                roll as i32, angles[1] as i32, angles[2] as i32);
                        }
                    },
                    Err(_) => {
                        if read_count % 100 == 0 {
                            info!("IMU read error");
                        }
                    }
                }
            }
        }
        
        // Wait for next sensor reading cycle - 10ms for 100Hz
        Timer::after(Duration::from_millis(10)).await;
    }
}

// Motor control task - handles motor movement based on IMU data
#[embassy_executor::task]
async fn motor_control_task() {
    info!("Motor control task started");
    
    // Dead zone to prevent jittering when nearly balanced
    const DEAD_ZONE_DEGREES: f32 = 1.0;
    
    // Track the step count for logging
    let mut step_count = 0;
    
    // Motor control loop - higher frequency (500Hz = 2ms interval)
    loop {
        let current_time = Instant::now().duration_since_epoch().as_micros() as u64;
        
        unsafe {
            if let (Some(dir_pin_ref), Some(step_pin_ref)) = (&mut DIR_PIN, &mut STEP_PIN) {
                // Create a new motor instance inside the loop
                let dir_pin_wrapped = OutputWrapper::new(dir_pin_ref);
                let step_pin_wrapped = OutputWrapper::new(step_pin_ref);
                let mut motor = StepperMotor::new_esp32(dir_pin_wrapped, step_pin_wrapped);
                
                // Get roll from shared atomic
                let roll_int = ROLL_ANGLE.load(Ordering::Relaxed);
                let roll = (roll_int as f32) / 100.0;
                
                if roll.abs() > DEAD_ZONE_DEGREES {
                    // Calculate a smoother motor speed with gradual changes
                    // Use non-linear gain that increases with angle magnitude
                    let base_gain = 60.0;  // Base gain for smoother motion
                    let adaptive_gain = base_gain * (1.0 + roll.abs() * 0.3);
                    
                    // Calculate target motor speed based on current roll angle
                    let target_speed = -roll * adaptive_gain; // Negative roll for correct direction
                    
                    // Apply speed directly since we're creating a new motor instance each time
                    let max_speed = 1500.0;
                    let clamped_speed = target_speed.abs().min(max_speed);
                    motor.set_speed(if target_speed > 0.0 { clamped_speed } else { -clamped_speed });
                    
                    // Execute motor step
                    match motor.move_continuous(current_time) {
                        Ok(stepped) => {
                            if stepped {
                                step_count += 1;
                                // Log occasionally
                                if step_count % 500 == 0 {
                                    info!(
                                        "Motor: speed={}, roll={}, pos={}",
                                        target_speed as i32,
                                        roll as i32,
                                        motor.get_position()
                                    );
                                }
                            }
                        },
                        Err(_) => {
                            if step_count % 500 == 0 {
                                info!("Motor step error");
                            }
                        }
                    }
                    
                    // Store current motor speed for monitoring
                    MOTOR_SPEED.store((motor.get_current_speed() * 10.0) as i32, Ordering::Relaxed);
                    
                    // Update shared motor position
                    MOTOR_POSITION.store(motor.get_position(), Ordering::Relaxed);
                } else {
                    // When in the dead zone, just stop the motor
                    motor.set_speed(0.0);
                    MOTOR_SPEED.store(0, Ordering::Relaxed);
                    
                    if step_count % 500 == 0 {
                        info!("In dead zone - motor stopped");
                    }
                }
            }
        }
        
        // Short delay for precise timing - 2ms for 500Hz
        Timer::after(Duration::from_millis(2)).await;
    }
}