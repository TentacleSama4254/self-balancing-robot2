#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Io, Level, Output, OutputConfig};
use esp_hal::i2c::master::I2c;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::time::Instant;
use esp_println as _;
use self_balancing_robot2::imu::FreeSixIMU;
use self_balancing_robot2::i2c_wrapper::I2cWrapper;
use self_balancing_robot2::motor::stepper::{OutputWrapper, StepperMotor};
use core::cell::RefCell;
use core::sync::atomic::{AtomicI32, Ordering};
use static_cell::StaticCell;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

const CALIBRATE_IMU: bool = false;

// Global static variables for sharing data between tasks
static ROLL_ANGLE: Mutex<CriticalSectionRawMutex, f32> = Mutex::new(0.0);
static MOTOR_POSITION: AtomicI32 = AtomicI32::new(0);

// Static storage for our hardware
// Define the generic type for I2C
type I2cType = esp_hal::i2c::master::I2c<'static, esp_hal::peripherals::I2C0>;
static IMU: StaticCell<FreeSixIMU<I2cWrapper<'static, I2cType>>> = StaticCell::new();
static DIR_PIN: StaticCell<Output<'static>> = StaticCell::new();
static STEP_PIN: StaticCell<Output<'static>> = StaticCell::new();
static DIR_WRAP: StaticCell<OutputWrapper<'static>> = StaticCell::new();
static STEP_WRAP: StaticCell<OutputWrapper<'static>> = StaticCell::new();
static MOTOR: StaticCell<StepperMotor<OutputWrapper<'static>, OutputWrapper<'static>>> = StaticCell::new();

// This creates a default app-descriptor required by the esp-idf bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    info!("Starting self-balancing robot with IMU and async tasks...");

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    
    // Configure I2C pins 
    let io25scl = peripherals.GPIO25;
    let io33sda = peripherals.GPIO33;

    // Configure TMC2209 stepper driver pins
    let io = Io::new(peripherals.IO_MUX);

    // Initialize pins and move to static storage
    let dir_pin = DIR_PIN.init(Output::new(peripherals.GPIO18, Level::Low, OutputConfig::default()));
    let step_pin = STEP_PIN.init(Output::new(peripherals.GPIO19, Level::Low, OutputConfig::default()));
    
    // Create wrappers and store them statically
    let dir_wrapper = DIR_WRAP.init(OutputWrapper::new(dir_pin));
    let step_wrapper = STEP_WRAP.init(OutputWrapper::new(step_pin));
    
    // Create stepper motor with wrappers (we need to dereference the static cells)
    let dir_wrapper_ref = &*dir_wrapper;
    let step_wrapper_ref = &*step_wrapper;
    let mut motor = StepperMotor::new_esp32(dir_wrapper_ref.clone(), step_wrapper_ref.clone());
    
    // Configure motor for maximum responsiveness
    motor.set_acceleration(4000.0); // Ultra-high acceleration for immediate response
    motor.set_speed(1000.0);        // Higher initial speed
    
    // Store motor in static cell
    let motor = MOTOR.init(motor);
    
    info!("Stepper motor initialized with high-speed settings!");
    
    // Initialize I2C for IMU
    let i2c = I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default(),
    )
    .unwrap()
    .with_sda(io33sda)
    .with_scl(io25scl);
    
    // Create I2C wrapper with static lifetime
    let i2c_cell = RefCell::new(i2c);
    let i2c_wrapper = I2cWrapper::new(&i2c_cell);
    
    // Store IMU in static storage
    let mut imu = FreeSixIMU::new(i2c_wrapper);
    
    // Initialize timer for embassy
    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);
    
    info!("Embassy initialized!");

    // Define a delay function for initialization
    let mut delay_fn = |ms| {
        // Basic delay implementation
        let cycles_per_ms = 240_000; // ESP32 at ~240MHz
        for _ in 0..ms * cycles_per_ms {
            core::hint::spin_loop();
        }
    };
    
    // Initialize IMU
    match imu.init(&mut delay_fn) {
        Ok(_) => info!("IMU initialized successfully!"),
        Err(_) => info!("Failed to initialize IMU!"),
    };
    
    // Skip calibration in this version
    info!("Skipping IMU calibration for quick startup.");

    // Move IMU to static storage
    let imu = IMU.init(imu);
    
    // Spawn our async tasks
    info!("Spawning async tasks...");
    spawner.spawn(imu_task()).unwrap();
    spawner.spawn(motor_task()).unwrap();
    
    // Main loop for system monitoring
    info!("All tasks spawned, entering monitoring loop");
    
    let mut counter = 0u64;
    loop {
        counter += 1;
        
        if counter % 20 == 0 {
            // Every ~2 seconds, report system status
            let roll = *ROLL_ANGLE.lock().await;
            let motor_pos = MOTOR_POSITION.load(Ordering::Relaxed);
            
            info!("System monitor: Roll={}, Motor position={}", roll as i32, motor_pos);
        }
        
        // Sleep for 100ms - main loop just monitors and doesn't need to be frequent
        Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::task]
async fn imu_task() {
    info!("IMU task started!");
    
    // Safe access to static IMU - embassy ensures tasks have exclusive access to their resources
    let imu = unsafe { &mut *IMU.get() };
    
    let mut interval_counter = 0u64;
    
    loop {
        // Get current time for IMU calculations
        let current_time = Instant::now().duration_since_epoch().as_micros() as u64;
        
        // Read IMU values
        match imu.get_euler_angles(current_time) {
            Ok(angles) => {
                // Only the roll is needed for balance control
                let roll = angles[0];
                
                // Store roll angle in shared state for motor task to use
                let mut roll_lock = ROLL_ANGLE.lock().await;
                *roll_lock = roll;
                
                // Print data periodically
                interval_counter += 1;
                if interval_counter % 20 == 0 {  // Reduce logging - every 20th reading
                    info!("IMU: Roll={}, Pitch={}, Yaw={} degrees", 
                        roll as i32, angles[1] as i32, angles[2] as i32);
                }
            }
            Err(_) => {
                if interval_counter % 100 == 0 {
                    info!("Failed to read euler angles");
                }
            }
        }
        
        // Running at 100Hz (10ms intervals) for smooth readings
        Timer::after(Duration::from_millis(10)).await;
    }
}

#[embassy_executor::task]
async fn motor_task() {
    info!("Motor control task started!");
    
    // Safe access to static motor - embassy ensures tasks have exclusive access to their resources
    let motor = unsafe { &mut *MOTOR.get() };
    
    let mut interval_counter = 0u64;
    
    loop {
        // Get current time for motor timing
        let current_time = Instant::now().duration_since_epoch().as_micros() as u64;
        
        // Get current roll angle from shared state
        let roll = *ROLL_ANGLE.lock().await;
        
        // Update motor position in shared state for monitoring
        MOTOR_POSITION.store(motor.get_position(), Ordering::Relaxed);
        
        // Ultra-sensitive threshold for immediate response
        if roll.abs() > 0.2 {  // Even more sensitive threshold (was 0.3)
            // Calculate motor speed with variable gain for more dramatic movement
            // Use a non-linear gain that increases with angle magnitude
            let base_gain = 100.0;  // Base gain (was 80.0)
            let adaptive_gain = base_gain * (1.0 + roll.abs() * 0.5);  // Gain increases with angle
            
            let motor_speed = roll * adaptive_gain;
            motor.set_speed(motor_speed.abs().min(2000.0));  // Higher speed cap for more dramatic movement
            
            // Determine step direction based on roll angle
            let step_direction = if roll > 0.0 { -1 } else { 1 };
            
            // Enhanced step calculation with progressive response
            // Small angles: take a few steps
            // Large angles: take many more steps for dramatic movement
            let base_steps = 2.0;  // Minimum multiplier
            let angle_factor = roll.abs() * 8.0;  // Increased from 5.0 to 8.0
            let steps_to_take = (base_steps + angle_factor).ceil() as i32;
            let step_direction = step_direction * steps_to_take.max(1);
            
            // Execute the step
            match motor.move_steps(step_direction, current_time) {
                Ok(stepped) => {
                    interval_counter += 1;
                    if stepped && interval_counter % 20 == 0 {  // Reduce logging frequency
                        info!("Motor: dir={}, steps={}, roll={}",
                            if step_direction > 0 { 1 } else { -1 },
                            if step_direction > 0 { step_direction } else { -step_direction },
                            roll as i32);
                    }
                },
                Err(_) => {
                    if interval_counter % 100 == 0 {
                        info!("Motor step error");
                    }
                }
            }
        } else {
            // If roll is small, stop the motor
            motor.set_speed(0.0);
            
            interval_counter += 1;
            if interval_counter % 100 == 0 {
                info!("Motor idle - roll angle too small");
            }
        }
        
        // Running at 250Hz (4ms intervals) for ultra-responsive motor control
        Timer::after(Duration::from_millis(4)).await;
    }
}
