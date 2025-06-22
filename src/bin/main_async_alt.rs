#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Io, Level, Output, OutputConfig};
use esp_hal::i2c::master::I2c;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::time::Instant;
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

// Shared data structure for roll angle
struct MotorCommand {
    steps: i32,
    roll: f32,
}

// Global shared channels for inter-task communication
static MOTOR_COMMAND: Signal<CriticalSectionRawMutex, MotorCommand> = Signal::new();
static ROLL_ANGLE: Mutex<CriticalSectionRawMutex, f32> = Mutex::new(0.0);

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

    // Initialize pins
    let mut dir_pin = Output::new(peripherals.GPIO18, Level::Low, OutputConfig::default());
    let mut step_pin = Output::new(peripherals.GPIO19, Level::Low, OutputConfig::default());
    
    // Create the TMC2209 stepper motor driver with enhanced settings
    let dir_pin_wrapped = self_balancing_robot2::motor::stepper::OutputWrapper::new(&mut dir_pin);
    let step_pin_wrapped = self_balancing_robot2::motor::stepper::OutputWrapper::new(&mut step_pin);
    let mut stepper_motor = StepperMotor::new_esp32(dir_pin_wrapped, step_pin_wrapped);
    stepper_motor.set_acceleration(4000.0); // Ultra-high acceleration for immediate response
    stepper_motor.set_speed(1000.0);        // Higher initial speed for more dramatic movement
    
    info!("Stepper motor initialized with high-speed settings!");
    
    // Initialize I2C for IMU
    let i2c = I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default(),
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

    // Define a delay function for initialization
    let mut delay_fn = |ms| {
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

    // Spawn our worker tasks
    spawner.spawn(imu_task(imu)).unwrap();
    spawner.spawn(motor_task(stepper_motor)).unwrap();
    
    info!("All tasks spawned, entering monitoring loop");
    
    let mut counter = 0u64;
    loop {
        counter += 1;
        
        if counter % 10 == 0 {
            // Every second, report system status
            let roll = *ROLL_ANGLE.lock().await;
            
            info!("System monitor: Roll={}", roll as i32);
        }
        
        // Sleep for 100ms
        Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::task]
async fn imu_task(mut imu: FreeSixIMU<I2cWrapper<'_, esp_hal::i2c::master::I2c<'_, esp_hal::peripherals::I2C0>>>) {
    info!("IMU task started!");
    
    let mut interval_counter = 0u64;
    
    loop {
        // Get current time for IMU calculations
        let current_time = Instant::now().duration_since_epoch().as_micros() as u64;
        
        // Read IMU values
        match imu.get_euler_angles(current_time) {
            Ok(angles) => {
                // Only the roll is needed for balance control
                let roll = angles[0];
                
                // Store roll angle in shared mutex for monitoring
                {
                    let mut roll_lock = ROLL_ANGLE.lock().await;
                    *roll_lock = roll;
                }
                
                // Calculate steps to move based on roll angle with enhanced sensitivity
                if roll.abs() > 0.2 {  // Even more sensitive threshold (was 0.3)
                    // Determine step direction based on roll
                    let step_direction = if roll > 0.0 { -1 } else { 1 };
                    
                    // Enhanced step calculation with progressive response
                    // Small angles: take a few steps
                    // Large angles: take many more steps for dramatic movement
                    let base_steps = 2.0;  // Minimum multiplier
                    let angle_factor = roll.abs() * 8.0;  // Increased from 5.0 to 8.0
                    let steps_to_take = (base_steps + angle_factor).ceil() as i32;
                    let steps = step_direction * steps_to_take.max(1);
                    
                    // Send motor command through signal
                    MOTOR_COMMAND.signal(MotorCommand {
                        steps,
                        roll,
                    }).await;
                }
                
                // Print data periodically
                interval_counter += 1;
                if interval_counter % 20 == 0 {
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
async fn motor_task(mut motor: StepperMotor<
    self_balancing_robot2::motor::stepper::OutputWrapper<'_>,
    self_balancing_robot2::motor::stepper::OutputWrapper<'_>
>) {
    info!("Motor control task started!");
    
    let mut interval_counter = 0u64;
    
    loop {
        // Get current time for motor timing
        let current_time = Instant::now().duration_since_epoch().as_micros() as u64;
        
        // Check for a new motor command (non-blocking)
        if let Some(command) = MOTOR_COMMAND.try_take() {
            // Calculate motor speed with variable gain for more dramatic movement
            let base_gain = 100.0;  // Base gain (was 80.0)
            let adaptive_gain = base_gain * (1.0 + command.roll.abs() * 0.5);  // Gain increases with angle
            
            let motor_speed = command.roll * adaptive_gain;
            motor.set_speed(motor_speed.abs().min(2000.0));
            
            // Execute the motor step
            match motor.move_steps(command.steps, current_time) {
                Ok(stepped) => {
                    if stepped && interval_counter % 20 == 0 {
                        info!("Motor: dir={}, steps={}, roll={}",
                            if command.steps > 0 { 1 } else { -1 },
                            if command.steps > 0 { command.steps } else { -command.steps },
                            command.roll as i32);
                    }
                },
                Err(_) => {
                    if interval_counter % 100 == 0 {
                        info!("Motor step error");
                    }
                }
            }
        } else if interval_counter % 100 == 0 {
            // No command received, periodically report idle status
            motor.set_speed(0.0);  // Ensure motor doesn't move when idle
        }
        
        interval_counter += 1;
        
        // Running at 250Hz (4ms intervals) for ultra-responsive motor control
        Timer::after(Duration::from_millis(4)).await;
    }
}
