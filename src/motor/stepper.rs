use embedded_hal_0_2::digital::v2::OutputPin;
use esp_hal::gpio;
// Using OutputPin trait for compatibility with different pin driver implementations

const MICROSTEPS: u16 = 32; // 1/32 microstepping
const STEPS_PER_REVOLUTION: u16 = 200; // NEMA 17 typically has 200 steps per revolution

// Create wrapper types for esp-hal Output pins
pub struct OutputWrapper<'a> {
    pin: &'a mut gpio::Output<'a>,
}

impl<'a> OutputWrapper<'a> {
    pub fn new(pin: &'a mut gpio::Output<'a>) -> Self {
        Self { pin }
    }
}

// Implementation to convert esp-hal Output pins to embedded-hal 0.2 compatible pins
impl<'a> OutputPin for OutputWrapper<'a> {
    type Error = core::convert::Infallible;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.set_low();
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.set_high();
        Ok(())
    }
}

/// TMC2209 Stepper Motor Driver
pub struct StepperMotor<DIR, STEP> {
    /// Direction pin
    dir_pin: DIR,
    
    /// Step pin
    step_pin: STEP,
    
    /// Current direction (true = forward, false = reverse)
    direction: bool,
    
    /// Current position in microsteps
    position: i32,
    
    /// Speed in steps per second
    speed: f32,
    
    /// Acceleration in steps per second^2
    acceleration: f32,
    
    /// Last step time in microseconds 
    last_step_time: u64,
    
    /// Minimum delay between steps based on max speed
    min_step_delay_micros: u64,
}

impl<DIR, STEP, E1, E2> StepperMotor<DIR, STEP>
where
    DIR: OutputPin<Error = E1>,
    STEP: OutputPin<Error = E2>,
{
    /// Create a new stepper motor instance
    pub fn new(dir_pin: DIR, step_pin: STEP) -> Self {
        Self {
            dir_pin,
            step_pin,
            direction: true, // Default to forward
            position: 0,
            speed: 0.0,
            acceleration: 800.0, // Default acceleration (steps/s^2)
            last_step_time: 0,
            min_step_delay_micros: 1000, // Default to 1ms between steps (1000 steps/s max)
        }
    }
    
    /// Set the direction pin
    pub fn set_direction(&mut self, forward: bool) -> Result<(), E1> {
        if forward {
            self.dir_pin.set_high()?;
        } else {
            self.dir_pin.set_low()?;
        }
        self.direction = forward;
        Ok(())
    }
    
    /// Generate a step pulse
    pub fn step(&mut self) -> Result<(), E2> {
        self.step_pin.set_high()?;
        // Need a small delay here for the pulse to be recognized
        // We'll handle this in the step control loop
        self.step_pin.set_low()?;
        
        // Update position based on direction
        if self.direction {
            self.position += 1;
        } else {
            self.position -= 1;
        }
        
        Ok(())
    }
    
    /// Set the motor speed in steps per second
    pub fn set_speed(&mut self, speed: f32) {
        self.speed = speed;
        if speed != 0.0 {
            // Calculate minimum step delay in microseconds
            self.min_step_delay_micros = (1_000_000.0 / speed.abs()) as u64;
        }
    }
    
    /// Set the motor acceleration in steps per second^2
    pub fn set_acceleration(&mut self, acceleration: f32) {
        self.acceleration = acceleration;
    }
    
    /// Check if it's time to make a step based on speed and last step time
    pub fn should_step(&self, current_time_micros: u64) -> bool {
        if self.speed == 0.0 {
            return false;
        }
        
        current_time_micros - self.last_step_time >= self.min_step_delay_micros
    }
    
    /// Update the last step time
    pub fn update_last_step_time(&mut self, time_micros: u64) {
        self.last_step_time = time_micros;
    }
    
    /// Move a specified number of steps
    pub fn move_steps(&mut self, steps: i32, current_time_micros: u64) -> Result<bool, E2> {
        // Set direction based on step sign
        let _ = self.set_direction(steps >= 0);
        
        // Check if it's time to step
        if self.should_step(current_time_micros) {
            self.step()?;
            self.update_last_step_time(current_time_micros);
            return Ok(true); // Step taken
        }
        
        Ok(false) // No step taken
    }
    
    /// Get the current position in steps
    pub fn get_position(&self) -> i32 {
        self.position
    }
    
    /// Set the current position (useful for homing/resetting)
    pub fn set_position(&mut self, position: i32) {
        self.position = position;
    }
    
    /// Get steps per degree considering microstepping
    pub fn steps_per_degree(&self) -> f32 {
        (STEPS_PER_REVOLUTION as f32 * MICROSTEPS as f32) / 360.0
    }
    
    /// Move to a specific angle in degrees
    pub fn move_to_angle(&mut self, angle: f32, current_time_micros: u64) -> Result<bool, E2> {
        let target_position = (angle * self.steps_per_degree()) as i32;
        let steps_to_move = target_position - self.position;
        
        if steps_to_move == 0 {
            return Ok(false); // Already at target position
        }
        
        self.move_steps(steps_to_move.signum(), current_time_micros)
    }
    
    /// Balance assist - move the motor based on gyroscope reading
    /// For self-balancing, we need to counter-act the tilt detected by the IMU
    pub fn balance_control(&mut self, gyro_angle: f32, current_time_micros: u64) -> Result<bool, E2> {
        // Simple proportional control - adjust speed based on tilt angle
        // A more sophisticated controller would use PID control

        // Higher angles need faster correction
        let desired_speed = gyro_angle * 10.0; // 10 steps/s per degree of tilt
        self.set_speed(desired_speed);
        
        // Move in the direction that would counter the tilt
        let steps_to_move = if desired_speed > 0.0 { 1 } else if desired_speed < 0.0 { -1 } else { 0 };
        
        if steps_to_move == 0 {
            return Ok(false); // No movement needed
        }
        
        self.move_steps(steps_to_move, current_time_micros)
    }
}

/// TMC2209 Stepper Motor Driver implementation for ESP32
impl<DIR, STEP> StepperMotor<DIR, STEP> 
where
    DIR: OutputPin,
    STEP: OutputPin,
{
    /// Create a new ESP32-specific stepper motor instance
    pub fn new_esp32(dir_pin: DIR, step_pin: STEP) -> Self {
        Self {
            dir_pin,
            step_pin,
            direction: true, // Default to forward
            position: 0,
            speed: 0.0,
            acceleration: 800.0, // Default acceleration
            last_step_time: 0,
            min_step_delay_micros: 1000, // Default to 1ms between steps (1000 steps/s max)
        }
    }
    
    /// Generate a step pulse with precise timing for ESP32
    pub fn step_with_delay(&mut self, delay_micros: u32) -> Result<(), ()> {
        // Set step pin high
        let _ = self.step_pin.set_high();
        
        // Small delay for pulse width
        // For TMC2209, minimum pulse width is typically 1μs
        // Using the FnMut pattern rather than direct delay to match the rest of the codebase
        // We just need to sleep for a bit, core::hint::spin_loop() would also work
        // but this is a very short delay anyway
        
        // Set step pin low
        let _ = self.step_pin.set_low();
        
        // For the delay between steps, we'll just use spin loops which is simple
        // and works for our purposes. In a more sophisticated implementation,
        // we might want to use a more proper timing mechanism.
        for _ in 0..delay_micros * 240 { // Assuming ~240MHz clock
            core::hint::spin_loop();
        }
        
        // Update position based on direction
        if self.direction {
            self.position += 1;
        } else {
            self.position -= 1;
        }
        
        Ok(())
    }
}
