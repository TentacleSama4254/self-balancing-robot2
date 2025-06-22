use embedded_hal_0_2::digital::v2::OutputPin;
use esp_hal::gpio;
// Using OutputPin trait for compatibility with different pin driver implementations

const MICROSTEPS: u16 = 32; // 1/32 microstepping
const STEPS_PER_REVOLUTION: u16 = 200; // NEMA 17 typically has 200 steps per revolution

// Using PhantomData for lifetime management
use core::marker::PhantomData;

// Create wrapper type for esp-hal Output pins using raw pointers for safe internal operations
#[derive(Copy, Clone)]
pub struct OutputWrapper<'a> {
    pin_ptr: *mut gpio::Output<'a>,
    // PhantomData to track lifetime without owning the reference
    _phantom: PhantomData<&'a mut gpio::Output<'a>>,
}

impl<'a> OutputWrapper<'a> {
    pub fn new(pin: &'a mut gpio::Output<'a>) -> Self {
        Self { 
            pin_ptr: pin as *mut gpio::Output<'a>,
            _phantom: PhantomData,
        }
    }

    // Add method to update the pin reference - useful for 'static lifetime references
    pub fn update_pin(&mut self, pin: &'a mut gpio::Output<'a>) {
        self.pin_ptr = pin as *mut gpio::Output<'a>;
    }
    
    // Safe method to get mutable access to the pin
    fn pin(&mut self) -> &mut gpio::Output<'a> {
        // SAFETY: We ensure the lifetime 'a is maintained and the pointer remains valid
        // through our implementation. The mutable reference is only accessible through
        // methods that take &mut self, so we maintain exclusivity.
        unsafe { &mut *self.pin_ptr }
    }
}

// Implementation to convert esp-hal Output pins to embedded-hal 0.2 compatible pins
impl<'a> OutputPin for OutputWrapper<'a> {
    type Error = core::convert::Infallible;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin().set_low();
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin().set_high();
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
    
    /// Minimum pulse width in cycles (affects step pulse duration)
    min_pulse_width: u32,
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
            min_pulse_width: 10, // Default minimum pulse width (in cycles)
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
        // Clamp speed to a realistic maximum (2000 steps/s is quite fast but achievable)
        let max_speed = 2000.0;
        let clamped_speed = if speed.abs() > max_speed { 
            if speed > 0.0 { max_speed } else { -max_speed }
        } else {
            speed
        };
        
        self.speed = clamped_speed;
        
        if clamped_speed != 0.0 {
            // Calculate minimum step delay in microseconds with a minimum to prevent too rapid stepping
            let calculated_delay = (1_000_000.0 / clamped_speed.abs()) as u64;
            self.min_step_delay_micros = calculated_delay.max(100); // Minimum 100μs delay (10,000 steps/s theoretical max)
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
            // For absolute values greater than 1, we'll take multiple steps to make movement more responsive
            let steps_to_take = steps.abs().min(5); // Limit to 5 steps at once for safety
            
            for _ in 0..steps_to_take {
                self.step()?;
                
                // Small delay between steps for stability if taking multiple steps
                if steps_to_take > 1 {
                    // Brief delay - just enough to register as separate steps
                    for _ in 0..100 {
                        core::hint::spin_loop();
                    }
                }
            }
            
            self.update_last_step_time(current_time_micros);
            return Ok(true); // Step(s) taken
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
    
    /// Set a minimum pulse width for the step signal in microseconds
    /// This ensures the stepper driver registers the step pulse correctly
    pub fn set_min_pulse_width(&mut self, width_micros: u64) {
        // Clamp to a reasonable range
        let clamped_width = width_micros.max(1).min(1000);
        
        // Update the minimum step delay to accommodate the pulse width
        self.min_step_delay_micros = self.min_step_delay_micros.max(clamped_width);
    }
    
    /// Get the current speed setting
    pub fn get_current_speed(&self) -> f32 {
        self.speed
    }
    
    /// Move the motor continuously based on current speed setting
    /// This method is more suitable for smooth continuous motion
    pub fn move_continuous(&mut self, current_time_micros: u64) -> Result<bool, E2> {
        if self.speed == 0.0 {
            return Ok(false);
        }
        
        // Set the direction based on speed sign
        let _ = self.set_direction(self.speed > 0.0);
        
        // Check if it's time to step
        if self.should_step(current_time_micros) {
            self.step()?;
            self.update_last_step_time(current_time_micros);
            return Ok(true); // Step taken
        }
        
        Ok(false) // No step taken
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
            min_pulse_width: 10, // Default minimum pulse width (in cycles)
        }
    }
    
    /// Generate a step pulse with precise timing for ESP32
    pub fn step_with_delay(&mut self, delay_micros: u32) -> Result<(), ()> {
        // Set step pin high
        let _ = self.step_pin.set_high();
        
        // Use configurable minimum pulse width (TMC2209 needs ~1μs minimum)
        // This is critical for proper step recognition by the driver
        for _ in 0..self.min_pulse_width {
            core::hint::spin_loop();
        }
        
        // Set step pin low
        let _ = self.step_pin.set_low();
        
        // Adaptive delay based on requested speed
        // For smoother operation at higher speeds, we reduce the delay
        // but ensure it's never too short for the stepper driver to miss steps
        let adaptive_delay = match delay_micros {
            0..=20 => 20,        // Minimum safe delay (very fast)
            21..=100 => delay_micros * 2 / 3,  // Reduced delay for fast speeds
            101..=500 => delay_micros * 3 / 4, // Slightly reduced for medium speeds
            _ => delay_micros,   // Normal delay for slow speeds
        };
        
        // For the delay between steps, use a more efficient loop
        // The multiplier converts microseconds to approximate CPU cycles
        // Adjusted to match the example code's timing approach
        for _ in 0..adaptive_delay * 100 {
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
