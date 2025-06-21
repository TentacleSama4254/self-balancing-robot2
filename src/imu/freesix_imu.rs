use embedded_hal::i2c::I2c;
use libm::{atan2f, asinf, sqrtf};

use crate::imu::adxl345::ADXL345;
use crate::imu::adxl345_registers::ADXL345_ADDR;
use crate::imu::itg3200::ITG3200;
use crate::imu::itg3200_registers::ITG3200_ADDR;

/// Default addresses for sensors
pub const FIMU_ACC_ADDR: u8 = ADXL345_ADDR;
pub const FIMU_ITG3200_DEF_ADDR: u8 = ITG3200_ADDR;

/// FreeSixIMU driver that combines accelerometer and gyroscope
pub struct FreeSixIMU<I2C> {
    i2c: I2C,
    acc: ADXL345<I2C>,
    gyro: ITG3200<I2C>,
    
    // AHRS algorithm variables
    q0: f32,
    q1: f32, 
    q2: f32,
    q3: f32,
    ex_int: f32,
    ey_int: f32,
    ez_int: f32,
    two_kp: f32,
    two_ki: f32,
    last_update: u64,
    sample_freq: f32,

    /// Status of the IMU sensor
    pub status: SensorStatus,
    
    /// Number of consecutive read failures
    pub consecutive_errors: u16,
    
    /// Number of readings since last successful read
    pub readings_since_ok: u16,
    
    /// Number of times the sensor was detected as disconnected
    pub disconnect_count: u16,
    
    /// Number of readings that were detected as faulty
    pub faulty_reading_count: u16,
    
    /// Previous accelerometer readings for drift calculation
    pub prev_accel: [f32; 3],
    
    /// Previous gyro readings for drift calculation
    pub prev_gyro: [f32; 3],
    
    /// Running average of accelerometer change rate (for drift detection)
    pub accel_change_rate: [f32; 3],
    
    /// Running average of gyro change rate (for drift detection)
    pub gyro_change_rate: [f32; 3],
    
    /// Estimated gyroscope bias values (for drift correction)
    gyro_bias: [f32; 3],
    
    /// Error estimates for Kalman filtering
    kalman_error_estimates: [f32; 6],
    
    /// Previous filtered readings (after Kalman filtering)
    filtered_values: [f32; 6],
    
    /// Time of last reading in microseconds
    last_reading_time: u64,
    
    /// Whether advanced filtering is enabled
    advanced_filtering: bool,
}

// Default values for the AHRS algorithm
const TWO_KP_DEF: f32 = 2.0 * 0.5; // 2 * proportional gain
const TWO_KI_DEF: f32 = 2.0 * 0.1; // 2 * integral gain
const M_PI: f32 = core::f32::consts::PI;

impl<I2C, E> FreeSixIMU<I2C>
where
    I2C: I2c<Error = E>,
    I2C: Clone,
{
    pub fn new(i2c: I2C) -> Self {
        let acc = ADXL345::new_with_address(i2c.clone(), FIMU_ACC_ADDR);
        let gyro = ITG3200::new_with_address(i2c.clone(), FIMU_ITG3200_DEF_ADDR);
        
        Self {
            i2c,
            acc,
            gyro,
            q0: 1.0,
            q1: 0.0,
            q2: 0.0,
            q3: 0.0,
            ex_int: 0.0,
            ey_int: 0.0,
            ez_int: 0.0,
            two_kp: TWO_KP_DEF,
            two_ki: TWO_KI_DEF,
            last_update: 0,
            sample_freq: 100.0, // Default sample frequency
            status: SensorStatus::default(),
            consecutive_errors: 0,
            readings_since_ok: 0,
            disconnect_count: 0,
            faulty_reading_count: 0,
            prev_accel: [0.0, 0.0, 0.0],
            prev_gyro: [0.0, 0.0, 0.0],
            accel_change_rate: [0.0, 0.0, 0.0],
            gyro_change_rate: [0.0, 0.0, 0.0],
            gyro_bias: [0.0, 0.0, 0.0],
            kalman_error_estimates: [0.0; 6],
            filtered_values: [0.0; 6],
            last_reading_time: 0,
            advanced_filtering: false,
        }
        }
    
    /// Initialize both sensors
    pub fn init<D>(&mut self, delay_fn: &mut D) -> Result<(), E>
    where
        D: FnMut(u32),
    {
        self.init_with_settings(FIMU_ACC_ADDR, FIMU_ITG3200_DEF_ADDR, false, false, delay_fn)
    }
    
    /// Initialize both sensors with fast mode option
    pub fn init_with_fast_mode<D>(&mut self, fast_mode: bool, delay_fn: &mut D) -> Result<(), E>
    where 
        D: FnMut(u32),
    {
        self.init_with_settings(FIMU_ACC_ADDR, FIMU_ITG3200_DEF_ADDR, fast_mode, false, delay_fn)
    }
    
    /// Initialize both sensors with advanced filtering option
    pub fn init_with_advanced_filtering<D>(&mut self, delay_fn: &mut D) -> Result<(), E>
    where
        D: FnMut(u32),
    {
        self.init_with_settings(FIMU_ACC_ADDR, FIMU_ITG3200_DEF_ADDR, false, true, delay_fn)
    }
    
    /// Initialize with custom settings
    pub fn init_with_settings<D>(
        &mut self, 
        acc_addr: u8, 
        gyro_addr: u8, 
        _fast_mode: bool,
        use_advanced_filtering: bool,
        delay_fn: &mut D
    ) -> Result<(), E>
    where
        D: FnMut(u32),
    {
        // Initialize accelerometer
        self.acc = ADXL345::new_with_address(self.i2c.clone(), acc_addr);
        self.acc.init()?;
        
        // Initialize gyroscope with error handling
        self.gyro = ITG3200::new_with_address(self.i2c.clone(), gyro_addr);
        
        // Try to initialize up to 3 times in case of connection issues
        let mut gyro_init_success = false;
        for _ in 0..3 {
            match self.gyro.init() {
                Ok(_) => {
                    gyro_init_success = true;
                    break;
                }
                Err(_) => {
                    // Wait before retrying
                    delay_fn(50);
                    self.gyro = ITG3200::new_with_address(self.i2c.clone(), gyro_addr);
                }
            }
        }
        
        // If we couldn't initialize after 3 attempts, return error
        if !gyro_init_success {
            self.status = SensorStatus::Disconnected;
            return Err(self.gyro.init().unwrap_err());
        }
        
        // Wait for gyro to stabilize
        delay_fn(1000);
        
        // Calibrate the ITG3200
        match self.zero_calibrate(128, delay_fn) {
            Ok(_) => {},
            Err(e) => {
                self.status = SensorStatus::NeedsCalibration;
                return Err(e);
            }
        }
        
        // Enable advanced filtering if requested
        if use_advanced_filtering {
            self.enable_advanced_filtering();
        } else {
            self.disable_advanced_filtering();
        }
        
        Ok(())
    }
    
    /// Calibrate the gyroscope
    pub fn zero_calibrate<D>(&mut self, samples: u16, delay_fn: &mut D) -> Result<(), E>
    where
        D: FnMut(u32),
    {
        self.gyro.zero_calibrate(samples, delay_fn)
    }
    
    /// Calibrate both accelerometer and gyroscope
    pub fn calibrate<D>(&mut self, gyro_samples: u16, accel_samples: u16, delay_fn: &mut D) -> Result<(), E>
    where
        D: FnMut(u32),
    {
        // Use improved calibration methods for better drift reduction
        
        // First, calibrate gyroscope with enhanced method
        use crate::imu::improved_calibration::ImprovedCalibration;
        ImprovedCalibration::calibrate_gyro(&mut self.gyro, gyro_samples, delay_fn)?;
        
        // Then calibrate accelerometer with enhanced method
        ImprovedCalibration::calibrate_accelerometer(&mut self.acc, accel_samples, delay_fn)?;
        
        Ok(())
    }
    
    /// Calibrate accelerometer by assuming device is flat (X, Y near 0, Z near 1G)
    pub fn calibrate_accelerometer<D>(&mut self, samples: u16, delay_fn: &mut D) -> Result<(), E>
    where
        D: FnMut(u32),
    {
        let mut x_sum: f32 = 0.0;
        let mut y_sum: f32 = 0.0;
        let mut z_sum: f32 = 0.0;
        
        // For outlier detection
        let mut min_x = f32::MAX;
        let mut max_x = f32::MIN;
        let mut min_y = f32::MAX;
        let mut max_y = f32::MIN;
        let mut min_z = f32::MAX;
        let mut max_z = f32::MIN;
        
        // Collect samples
        for _ in 0..samples {
            let (x, y, z) = self.acc.read_accel_g()?;
            x_sum += x;
            y_sum += y;
            z_sum += z;
            
            // Track min/max values for outlier detection
            min_x = min_x.min(x);
            max_x = max_x.max(x);
            min_y = min_y.min(y);
            max_y = max_y.max(y);
            min_z = min_z.min(z);
            max_z = max_z.max(z);
            
            // Small delay between samples
            delay_fn(5); // 5ms delay
        }
        
        // Calculate average values
        let x_avg = x_sum / samples as f32;
        let y_avg = y_sum / samples as f32;
        let z_avg = z_sum / samples as f32;
        
        // Check for excessive jitter during calibration (which would invalidate the results)
        let x_range = max_x - min_x;
        let y_range = max_y - min_y;
        let z_range = max_z - min_z;
        
        if x_range > 0.2 || y_range > 0.2 || z_range > 0.2 {
            // Too much jitter during calibration, use conservative calibration
            // Still apply some calibration, but don't fully trust these values
            if x_avg.abs() < 0.3 && y_avg.abs() < 0.3 && (z_avg - 1.0).abs() < 0.3 {
                // Apply moderate gains
                let x_gain = if x_avg.abs() > 0.01 { 0.5 * (0.0 - x_avg) / x_avg + 1.0 } else { 1.0 };
                let y_gain = if y_avg.abs() > 0.01 { 0.5 * (0.0 - y_avg) / y_avg + 1.0 } else { 1.0 };
                let z_gain = if z_avg.abs() > 0.01 { 0.5 * (1.0 - z_avg) / z_avg + 1.0 } else { 1.0 };
                
                self.acc.set_axis_gains(x_gain, y_gain, z_gain);
                
                // Also set hardware offsets if possible (convert to the range supported by hardware)
                // ADXL345 offsets are in raw LSB values, not g units
                let scale_factor = match self.acc.get_range_setting()? {
                    crate::imu::adxl345_registers::RANGE_2G => 3.9,
                    crate::imu::adxl345_registers::RANGE_4G => 7.8,
                    crate::imu::adxl345_registers::RANGE_8G => 15.6,
                    crate::imu::adxl345_registers::RANGE_16G => 31.2,
                    _ => 3.9, // Default to 2G if unknown
                };
                
                let x_offset = (-x_avg * 1000.0 / scale_factor).min(127.0).max(-128.0) as i8;
                let y_offset = (-y_avg * 1000.0 / scale_factor).min(127.0).max(-128.0) as i8;
                let z_offset = ((1.0 - z_avg) * 1000.0 / scale_factor).min(127.0).max(-128.0) as i8;
                
                self.acc.set_axis_offset(x_offset, y_offset, z_offset)?;
            }
        } else {
            // Good calibration data, apply full correction
            if x_avg.abs() < 0.3 && y_avg.abs() < 0.3 && (z_avg - 1.0).abs() < 0.3 {
                // First apply hardware offsets
                let scale_factor = match self.acc.get_range_setting()? {
                    crate::imu::adxl345_registers::RANGE_2G => 3.9,
                    crate::imu::adxl345_registers::RANGE_4G => 7.8,
                    crate::imu::adxl345_registers::RANGE_8G => 15.6,
                    crate::imu::adxl345_registers::RANGE_16G => 31.2,
                    _ => 3.9, // Default to 2G if unknown
                };
                
                let x_offset = (-x_avg * 1000.0 / scale_factor).min(127.0).max(-128.0) as i8;
                let y_offset = (-y_avg * 1000.0 / scale_factor).min(127.0).max(-128.0) as i8;
                let z_offset = ((1.0 - z_avg) * 1000.0 / scale_factor).min(127.0).max(-128.0) as i8;
                
                self.acc.set_axis_offset(x_offset, y_offset, z_offset)?;
                
                // Then apply fine-tuning gains - after reading new values with offsets applied
                // Small delay to allow settings to take effect
                delay_fn(10);
                
                // Re-read values after applying hardware offsets
                let mut new_x_sum: f32 = 0.0;
                let mut new_y_sum: f32 = 0.0;
                let mut new_z_sum: f32 = 0.0;
                
                for _ in 0..5 {
                    let (x, y, z) = self.acc.read_accel_g()?;
                    new_x_sum += x;
                    new_y_sum += y;
                    new_z_sum += z;
                    delay_fn(5);
                }
                
                let new_x_avg = new_x_sum / 5.0;
                let new_y_avg = new_y_sum / 5.0; 
                let new_z_avg = new_z_sum / 5.0;
                
                // Compute fine-tuning gains to get exact normalized accelerometer readings
                let x_gain = if new_x_avg.abs() > 0.01 { 0.0 / new_x_avg } else { 1.0 };
                let y_gain = if new_y_avg.abs() > 0.01 { 0.0 / new_y_avg } else { 1.0 };
                let z_gain = if new_z_avg.abs() > 0.01 { 1.0 / new_z_avg } else { 1.0 };
                
                self.acc.set_axis_gains(x_gain, y_gain, z_gain);
            }
        }
        
        Ok(())
    }
    
    /// Get raw sensor values
    pub fn get_raw_values(&mut self) -> Result<[i16; 6], E> {
        let (acc_x, acc_y, acc_z) = self.acc.read_accel()?;
        let (gyro_x, gyro_y, gyro_z) = self.gyro.read_gyro_raw()?;
        
        Ok([acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z])
    }
    
    /// Get converted sensor values (accelerometer in g, gyroscope in degrees/s)
    /// Also monitors sensor health, disconnections, and drift
    /// 
    /// Applies advanced filtering if enabled:
    /// - Kalman filtering to reduce noise
    /// - Adaptive bias estimation for gyro drift
    /// - Spike detection and filtering
    pub fn get_values(&mut self) -> Result<[f32; 6], E> {
        // First, check if the sensor is connected
        let disconnected = self.detect_disconnection()?;
        if disconnected {
            // Return zeros if disconnected
            return Ok([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        }
        
        // Try to read from both sensors
        let result = match (self.acc.read_accel_g(), self.gyro.read_gyro()) {
            (Ok((acc_x, acc_y, acc_z)), Ok((mut gyro_x, mut gyro_y, mut gyro_z))) => {
                // Auto-zero tiny gyro movements to reduce drift
                // These small values are likely just noise
                const GYRO_ZERO_THRESHOLD: f32 = 0.25;
                if gyro_x.abs() < GYRO_ZERO_THRESHOLD {
                    gyro_x = 0.0;
                }
                if gyro_y.abs() < GYRO_ZERO_THRESHOLD {
                    gyro_y = 0.0;
                }
                if gyro_z.abs() < GYRO_ZERO_THRESHOLD {
                    gyro_z = 0.0;
                }
                
                let raw_values = [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z];
                
                // Check for drift and faulty readings
                // The detect_drift method updates the sensor status internally
                self.detect_drift(raw_values);
                
                // Apply advanced filtering if enabled
                let values = if self.advanced_filtering {
                    self.apply_advanced_filtering(raw_values)
                } else {
                    raw_values
                };
                
                Ok(values)
            },
            (Ok((acc_x, acc_y, acc_z)), Err(_)) => {
                // Gyro failed but accelerometer worked
                self.consecutive_errors += 1;
                self.readings_since_ok += 1;
                self.status = SensorStatus::Faulty;
                Ok([acc_x, acc_y, acc_z, 0.0, 0.0, 0.0])
            },
            (Err(_), Ok((gyro_x, gyro_y, gyro_z))) => {
                // Accelerometer failed but gyro worked
                self.consecutive_errors += 1;
                self.readings_since_ok += 1;
                self.status = SensorStatus::Faulty;
                Ok([0.0, 0.0, 0.0, gyro_x, gyro_y, gyro_z])
            },
            (Err(_), Err(_)) => {
                // Both sensors failed
                self.consecutive_errors += 1;
                self.readings_since_ok += 1;
                
                if self.consecutive_errors >= 3 {
                    self.status = SensorStatus::Disconnected;
                    if self.readings_since_ok >= 10 {
                        self.disconnect_count += 1;
                    }
                } else {
                    self.status = SensorStatus::Faulty;
                }
                
                Ok([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            }
        };
        
        result
    }
    
    /// Update the quaternion (AHRS algorithm)
    fn ahrs_update(&mut self, gx: f32, gy: f32, gz: f32, ax: f32, ay: f32, az: f32, current_time: u64) {
        // Convert gyro values from degrees/sec to radians/sec
        let mut gx_rad = gx * M_PI / 180.0;
        let mut gy_rad = gy * M_PI / 180.0;
        let mut gz_rad = gz * M_PI / 180.0;
        
        // Calculate sample frequency
        if self.last_update != 0 {
            let dt = (current_time - self.last_update) as f32 / 1_000_000.0; // Convert micros to seconds
            if dt > 0.0 {
                self.sample_freq = 1.0 / dt;
            }
        }
        self.last_update = current_time;
        
        // Auxiliary variables to avoid repeated arithmetic
        let q0q0 = self.q0 * self.q0;
        let q0q1 = self.q0 * self.q1;
        let q0q2 = self.q0 * self.q2;
        // let _q0q3 = self.q0 * self.q3;  // Unused in current implementation
        // let _q1q1 = self.q1 * self.q1;  // Unused in current implementation
        // let _q1q2 = self.q1 * self.q2;  // Unused in current implementation
        let q1q3 = self.q1 * self.q3;
        // let _q2q2 = self.q2 * self.q2;  // Unused in current implementation
        let q2q3 = self.q2 * self.q3;
        let q3q3 = self.q3 * self.q3;
        
        let mut halfex = 0.0;
        let mut halfey = 0.0;
        let mut halfez = 0.0;
        
        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalization)
        if ax != 0.0 || ay != 0.0 || az != 0.0 {
            // Normalize accelerometer measurement
            let recipnorm = self.inv_sqrt(ax * ax + ay * ay + az * az);
            let ax_norm = ax * recipnorm;
            let ay_norm = ay * recipnorm;
            let az_norm = az * recipnorm;
            
            // Estimated direction of gravity and vector perpendicular to magnetic flux
            let halfvx = q1q3 - q0q2;
            let halfvy = q0q1 + q2q3;
            let halfvz = q0q0 - 0.5 + q3q3;
            
            // Error is sum of cross product between estimated direction and measured direction of gravity
            halfex = ay_norm * halfvz - az_norm * halfvy;
            halfey = az_norm * halfvx - ax_norm * halfvz;
            halfez = ax_norm * halfvy - ay_norm * halfvx;
        }
        
        // Apply feedback only when valid data has been gathered from the accelerometer
        if halfex != 0.0 || halfey != 0.0 || halfez != 0.0 {
            // Compute and apply integral feedback if enabled
            if self.two_ki > 0.0 {
                // Integral error scaled by Ki
                self.ex_int += self.two_ki * halfex * (1.0 / self.sample_freq);
                self.ey_int += self.two_ki * halfey * (1.0 / self.sample_freq);
                self.ez_int += self.two_ki * halfez * (1.0 / self.sample_freq);
                
                // Apply integral feedback
                gx_rad += self.ex_int;
                gy_rad += self.ey_int;
                gz_rad += self.ez_int;
            }
            
            // Apply proportional feedback
            gx_rad += self.two_kp * halfex;
            gy_rad += self.two_kp * halfey;
            gz_rad += self.two_kp * halfez;
        }
        
        // Integrate rate of change of quaternion
        let half_dt = 0.5 / self.sample_freq;
        let gx_half = gx_rad * half_dt;
        let gy_half = gy_rad * half_dt;
        let gz_half = gz_rad * half_dt;
        
        // Updated quaternion values
        let qa = self.q0;
        let qb = self.q1;
        let qc = self.q2;
        let qd = self.q3;
        
        self.q0 += -qb * gx_half - qc * gy_half - qd * gz_half;
        self.q1 += qa * gx_half + qc * gz_half - qd * gy_half;
        self.q2 += qa * gy_half - qb * gz_half + qd * gx_half;
        self.q3 += qa * gz_half + qb * gy_half - qc * gx_half;
        
        // Normalize quaternion
        let recipnorm = self.inv_sqrt(self.q0 * self.q0 + self.q1 * self.q1 + 
                                     self.q2 * self.q2 + self.q3 * self.q3);
        self.q0 *= recipnorm;
        self.q1 *= recipnorm;
        self.q2 *= recipnorm;
        self.q3 *= recipnorm;
    }
    
    /// Get the current quaternion
    pub fn get_quaternion(&mut self, current_time: u64) -> Result<[f32; 4], E> {
        let values = self.get_values()?;
        
        // Update AHRS algorithm (6 DOF version - no magnetometer)
        self.ahrs_update(
            values[3], values[4], values[5], // gyro values
            values[0], values[1], values[2], // accelerometer values
            current_time,
        );
        
        Ok([self.q0, self.q1, self.q2, self.q3])
    }
    
    /// Get Euler angles in degrees
    pub fn get_euler_angles(&mut self, current_time: u64) -> Result<[f32; 3], E> {
        let q = self.get_quaternion(current_time)?;
        
        // Convert quaternion to Euler angles (in radians)
        let roll = atan2f(2.0 * (q[0] * q[1] + q[2] * q[3]),
                         1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]));
                         
        let pitch = asinf(2.0 * (q[0] * q[2] - q[3] * q[1]));
        
        let yaw = atan2f(2.0 * (q[0] * q[3] + q[1] * q[2]),
                        1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
        
        // Convert from radians to degrees
        Ok([
            roll * 180.0 / M_PI,
            pitch * 180.0 / M_PI,
            yaw * 180.0 / M_PI
        ])
    }
    
    /// Get converted sensor values formatted with 3 decimal places
    pub fn get_formatted_values(&mut self) -> Result<([i32; 6], [u32; 6]), E> {
        let values = self.get_values()?;
        
        // Format to 3 decimal places (multiply by 1000 and separate integer and fractional parts)
        let mut int_parts = [0i32; 6];
        let mut frac_parts = [0u32; 6];
        
        for i in 0..6 {
            let scaled = values[i] * 1000.0;
            int_parts[i] = scaled as i32 / 1000;
            frac_parts[i] = (scaled.abs() as u32) % 1000;
        }
        
        Ok((int_parts, frac_parts))
    }
    
    /// Get Euler angles formatted with 3 decimal places
    pub fn get_formatted_euler_angles(&mut self, current_time: u64) -> Result<([i32; 3], [u32; 3]), E> {
        let angles = self.get_euler_angles(current_time)?;
        
        // Format to 3 decimal places (multiply by 1000 and separate integer and fractional parts)
        let mut int_parts = [0i32; 3];
        let mut frac_parts = [0u32; 3];
        
        for i in 0..3 {
            let scaled = angles[i] * 1000.0;
            int_parts[i] = scaled as i32 / 1000;
            frac_parts[i] = (scaled.abs() as u32) % 1000;
        }
        
        Ok((int_parts, frac_parts))
    }
    
    /// Fast inverse square-root
    /// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
    fn inv_sqrt(&self, x: f32) -> f32 {
        // Using standard library to maintain precision
        1.0 / sqrtf(x)
    }
}

/// Status of the IMU sensor
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorStatus {
    /// Functioning properly
    Ok,
    
    /// Recently disconnected or lost connection
    Disconnected,
    
    /// Connected but producing faulty readings
    Faulty,
    
    /// Connected but requires calibration
    NeedsCalibration,
    
    /// Currently in calibration process
    Calibrating,
    
    /// Sensor readings show excessive drift
    ExcessiveDrift,
}

impl Default for SensorStatus {
    fn default() -> Self {
        Self::Disconnected
    }
}

// Implement Format trait for SensorStatus to use with defmt::info!
impl defmt::Format for SensorStatus {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            SensorStatus::Ok => defmt::write!(fmt, "Ok"),
            SensorStatus::Disconnected => defmt::write!(fmt, "Disconnected"),
            SensorStatus::Faulty => defmt::write!(fmt, "Faulty"),
            SensorStatus::NeedsCalibration => defmt::write!(fmt, "NeedsCalibration"),
            SensorStatus::Calibrating => defmt::write!(fmt, "Calibrating"),
            SensorStatus::ExcessiveDrift => defmt::write!(fmt, "ExcessiveDrift"),
        }
    }
}

/// Structure to track sensor health and connection status
#[derive(Clone, Copy, Debug)]
pub struct SensorHealth {
    /// Current status of the sensor
    pub status: SensorStatus,
    
    /// Number of consecutive read failures
    pub consecutive_errors: u16,
    
    /// Number of readings since last successful read
    pub readings_since_ok: u16,
    
    /// Number of times the sensor was detected as disconnected
    pub disconnect_count: u16,
    
    /// Number of readings that were detected as faulty
    pub faulty_reading_count: u16,
    
    /// Previous accelerometer readings for drift calculation
    pub prev_accel: [f32; 3],
    
    /// Previous gyro readings for drift calculation
    pub prev_gyro: [f32; 3],
    
    /// Running average of accelerometer change rate (for drift detection)
    pub accel_change_rate: [f32; 3],
    
    /// Running average of gyro change rate (for drift detection)
    pub gyro_change_rate: [f32; 3],
}

impl Default for SensorHealth {
    fn default() -> Self {
        Self {
            status: SensorStatus::default(),
            consecutive_errors: 0,
            readings_since_ok: 0,
            disconnect_count: 0,
            faulty_reading_count: 0,
            prev_accel: [0.0, 0.0, 0.0],
            prev_gyro: [0.0, 0.0, 0.0],
            accel_change_rate: [0.0, 0.0, 0.0],
            gyro_change_rate: [0.0, 0.0, 0.0],
        }
    }
}

impl<I2C, E> FreeSixIMU<I2C>
where
    I2C: I2c<Error = E>,
    I2C: Clone,
{
    /// Auto-calibration that can be called during operation when the device is detected to be stationary
    pub fn auto_calibrate<D>(&mut self, _samples: u16, delay_fn: &mut D) -> Result<bool, E>
    where
        D: FnMut(u32),
    {
        // Update sensor status during calibration
        self.status = SensorStatus::Calibrating;
        
        // First, check if the device is actually stationary by measuring jitter
        let mut x_gyro_sum: f32 = 0.0;
        let mut y_gyro_sum: f32 = 0.0;
        let mut z_gyro_sum: f32 = 0.0;
        
        let mut x_accel_sum: f32 = 0.0;
        let mut y_accel_sum: f32 = 0.0;
        let mut z_accel_sum: f32 = 0.0;
        
        // We'll perform statistical analysis on the collected samples
        
        // For jitter detection
        let mut min_x_gyro = f32::MAX;
        let mut max_x_gyro = f32::MIN;
        let mut min_y_gyro = f32::MAX;
        let mut max_y_gyro = f32::MIN;
        let mut min_z_gyro = f32::MAX;
        let mut max_z_gyro = f32::MIN;
        
        // Collect a few samples to determine if the device is stationary
        const STABILITY_CHECK_SAMPLES: u16 = 5;
        
        for _ in 0..STABILITY_CHECK_SAMPLES {
            let values = self.get_values()?;
            
            x_accel_sum += values[0];
            y_accel_sum += values[1];
            z_accel_sum += values[2];
            
            x_gyro_sum += values[3];
            y_gyro_sum += values[4];
            z_gyro_sum += values[5];
            
            // Track min/max for gyro to detect movement
            min_x_gyro = min_x_gyro.min(values[3]);
            max_x_gyro = max_x_gyro.max(values[3]);
            min_y_gyro = min_y_gyro.min(values[4]);
            max_y_gyro = max_y_gyro.max(values[4]);
            min_z_gyro = min_z_gyro.min(values[5]);
            max_z_gyro = max_z_gyro.max(values[5]);
            
            delay_fn(5);
        }
        
        // Calculate average values
        let x_gyro_avg = x_gyro_sum / STABILITY_CHECK_SAMPLES as f32;
        let y_gyro_avg = y_gyro_sum / STABILITY_CHECK_SAMPLES as f32;
        let z_gyro_avg = z_gyro_sum / STABILITY_CHECK_SAMPLES as f32;
        
        let x_accel_avg = x_accel_sum / STABILITY_CHECK_SAMPLES as f32;
        let y_accel_avg = y_accel_sum / STABILITY_CHECK_SAMPLES as f32;
        let z_accel_avg = z_accel_sum / STABILITY_CHECK_SAMPLES as f32;
        
        // Calculate jitter range for gyro
        let x_gyro_jitter = max_x_gyro - min_x_gyro;
        let y_gyro_jitter = max_y_gyro - min_y_gyro;
        let z_gyro_jitter = max_z_gyro - min_z_gyro;
        
        // Check if the device is stationary - all gyro values should be near zero with minimal jitter
        const MAX_GYRO_JITTER: f32 = 0.5; // deg/s
        const MAX_GYRO_AVERAGE: f32 = 0.3; // deg/s
        
        if x_gyro_jitter > MAX_GYRO_JITTER || 
           y_gyro_jitter > MAX_GYRO_JITTER || 
           z_gyro_jitter > MAX_GYRO_JITTER ||
           x_gyro_avg.abs() > MAX_GYRO_AVERAGE ||
           y_gyro_avg.abs() > MAX_GYRO_AVERAGE ||
           z_gyro_avg.abs() > MAX_GYRO_AVERAGE {
            // Device is not stationary enough for calibration
            return Ok(false);
        }
        
        // Device appears to be stationary, proceed with gentle calibration
        
        // 1. Make small adjustments to gyro offsets
        if x_gyro_avg.abs() > 0.1 || y_gyro_avg.abs() > 0.1 || z_gyro_avg.abs() > 0.1 {
            // Get current offsets
            let (curr_x_offset, curr_y_offset, curr_z_offset) = self.gyro.get_offsets();
            
            // Calculate additional offset correction (convert from deg/s to raw values)
            let lsb_per_dps = 14.375; // ITG3200 sensitivity
            let x_additional = (x_gyro_avg * lsb_per_dps) as i16;
            let y_additional = (y_gyro_avg * lsb_per_dps) as i16;
            let z_additional = (z_gyro_avg * lsb_per_dps) as i16;
            
            // Apply refined offsets, but with smaller increments to avoid sudden jumps
            let adjustment_factor = 0.3; // Apply only 30% of the calculated adjustment for smoother transition
            self.gyro.set_offsets(
                curr_x_offset + (x_additional as f32 * adjustment_factor) as i16,
                curr_y_offset + (y_additional as f32 * adjustment_factor) as i16, 
                curr_z_offset + (z_additional as f32 * adjustment_factor) as i16
            );
        }
        
        // 2. Make small adjustments to accelerometer if needed
        if x_accel_avg.abs() > 0.05 || y_accel_avg.abs() > 0.05 || (z_accel_avg - 1.0).abs() > 0.05 {
            // Determine which axis is aligned with gravity to handle different orientations
            let gravity_aligned_x = x_accel_avg.abs() > 0.8;
            let gravity_aligned_y = y_accel_avg.abs() > 0.8;
            let gravity_aligned_z = z_accel_avg.abs() > 0.8;
            
            // Get current gains
            let curr_gains = self.acc.get_axis_gains();
            let curr_x_gain = curr_gains[0];
            let curr_y_gain = curr_gains[1];
            let curr_z_gain = curr_gains[2];
            
            // Calculate new gains based on orientation
            let mut new_x_gain = curr_x_gain;
            let mut new_y_gain = curr_y_gain;
            let mut new_z_gain = curr_z_gain;
            
            let adjustment_factor = 0.2; // 20% adjustment per auto-calibration
            
            if gravity_aligned_z {
                // Standard orientation (Z pointing down with gravity)
                let x_correction = -x_accel_avg * adjustment_factor;
                let y_correction = -y_accel_avg * adjustment_factor;
                let z_correction = (1.0 - z_accel_avg) * adjustment_factor;
                
                new_x_gain = curr_x_gain * (1.0 + x_correction);
                new_y_gain = curr_y_gain * (1.0 + y_correction);
                new_z_gain = curr_z_gain * (1.0 + z_correction);
            } else if gravity_aligned_x {
                // Device rotated 90 degrees (X pointing with/against gravity)
                let target = if x_accel_avg > 0.0 { 1.0 } else { -1.0 };
                let x_correction = (target - x_accel_avg) * adjustment_factor;
                let y_correction = -y_accel_avg * adjustment_factor;
                let z_correction = -z_accel_avg * adjustment_factor;
                
                new_x_gain = curr_x_gain * (1.0 + x_correction / target.abs());
                new_y_gain = curr_y_gain * (1.0 + y_correction);
                new_z_gain = curr_z_gain * (1.0 + z_correction);
            } else if gravity_aligned_y {
                // Device rotated 90 degrees (Y pointing with/against gravity)
                let target = if y_accel_avg > 0.0 { 1.0 } else { -1.0 };
                let x_correction = -x_accel_avg * adjustment_factor;
                let y_correction = (target - y_accel_avg) * adjustment_factor;
                let z_correction = -z_accel_avg * adjustment_factor;
                
                new_x_gain = curr_x_gain * (1.0 + x_correction);
                new_y_gain = curr_y_gain * (1.0 + y_correction / target.abs());
                new_z_gain = curr_z_gain * (1.0 + z_correction);
            }
            
            // Apply the new gains with limits to prevent extreme values
            new_x_gain = new_x_gain.max(0.8).min(1.2);
            new_y_gain = new_y_gain.max(0.8).min(1.2);
            new_z_gain = new_z_gain.max(0.8).min(1.2);
            
            self.acc.set_axis_gains(new_x_gain, new_y_gain, new_z_gain);
        }
        
        Ok(true)
    }
    
    /// Check if the sensor is disconnected
    pub fn detect_disconnection(&mut self) -> Result<bool, E> {
        // Try reading a register from both sensors
        let accelerometer_connected = self.acc.check_connection()?;
        let gyroscope_connected = self.gyro.check_connection()?;
        
        // Update sensor status and counters
        if !accelerometer_connected || !gyroscope_connected {
            self.consecutive_errors += 1;
            self.readings_since_ok += 1;
            
            // If we've had multiple consecutive failures, consider the sensor disconnected
            if self.consecutive_errors >= 3 {
                if self.status != SensorStatus::Disconnected {
                    self.disconnect_count += 1;
                }
                self.status = SensorStatus::Disconnected;
                return Ok(true);
            }
        } else {
            // Sensors responded correctly
            if self.status == SensorStatus::Disconnected {
                // If previously disconnected, now need calibration
                self.status = SensorStatus::NeedsCalibration;
            }
            self.consecutive_errors = 0;
            self.readings_since_ok = 0;
        }
        
        Ok(false)
    }
    
    /// Detect excessive drift or faulty readings in the IMU
    pub fn detect_drift(&mut self, values: [f32; 6]) -> bool {
        const ACCEL_CHANGE_THRESHOLD: f32 = 2.0;  // Max change rate g/s when stationary
        const GYRO_CHANGE_THRESHOLD: f32 = 15.0;  // Max change rate deg/s² when stationary
        const GRAVITY_THRESHOLD: f32 = 0.2;       // Max deviation from 1g total when stationary
        
        // Extract values
        let acc_x = values[0];
        let acc_y = values[1];
        let acc_z = values[2];
        // Gyro values are accessed directly through values array as needed
        
        // Check for impossible acceleration values
        // When stationary, the magnitude of the acceleration vector should be close to 1g
        let acc_magnitude = sqrtf(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
        let acc_magnitude_error = (acc_magnitude - 1.0).abs();
        
        if acc_magnitude_error > GRAVITY_THRESHOLD {
            self.faulty_reading_count += 1;
            self.status = SensorStatus::Faulty;
            return true;
        }
        
        // Check for excessive change rates - calculate derivatives
        let mut drifting = false;
        
        for i in 0..3 {
            // Calculate the rate of change for accelerometer values
            let accel_change_rate = (values[i] - self.prev_accel[i]).abs();
            
            // Update the running average of change rate with exponential smoothing
            self.accel_change_rate[i] = self.accel_change_rate[i] * 0.9 + accel_change_rate * 0.1;
            
            // Check if change rate is excessive
            if self.accel_change_rate[i] > ACCEL_CHANGE_THRESHOLD {
                drifting = true;
            }
            
            // Store the current value for next comparison
            self.prev_accel[i] = values[i];
        }
        
        for i in 0..3 {
            // Calculate the rate of change for gyro values
            let gyro_change_rate = (values[i+3] - self.prev_gyro[i]).abs();
            
            // Update the running average of change rate with exponential smoothing
            self.gyro_change_rate[i] = self.gyro_change_rate[i] * 0.9 + gyro_change_rate * 0.1;
            
            // Check if change rate is excessive
            if self.gyro_change_rate[i] > GYRO_CHANGE_THRESHOLD {
                drifting = true;
            }
            
            // Store the current value for next comparison
            self.prev_gyro[i] = values[i+3];
        }
        
        if drifting {
            self.status = SensorStatus::ExcessiveDrift;
            return true;
        }
        
        // Update status to OK if all checks passed
        if self.status != SensorStatus::Calibrating && self.status != SensorStatus::NeedsCalibration {
            self.status = SensorStatus::Ok;
        }
        
        false
    }

    /// Get the current sensor health status
    pub fn get_sensor_health(&self) -> SensorHealth {
        SensorHealth {
            status: self.status,
            consecutive_errors: self.consecutive_errors,
            readings_since_ok: self.readings_since_ok,
            disconnect_count: self.disconnect_count,
            faulty_reading_count: self.faulty_reading_count,
            prev_accel: self.prev_accel,
            prev_gyro: self.prev_gyro,
            accel_change_rate: self.accel_change_rate,
            gyro_change_rate: self.gyro_change_rate,
        }
    }

    /// Apply advanced filtering techniques to raw sensor values
    /// 
    /// - Kalman filtering for noise reduction
    /// - Adaptive bias estimation for gyroscope drift
    /// - Spike detection and filtering
    /// 
    /// Returns: Filtered sensor values
    fn apply_advanced_filtering(&mut self, raw_values: [f32; 6]) -> [f32; 6] {
        use crate::imu::drift_correction::DriftCorrection;
        
        let current_time = self.last_update; // Using AHRS last update time for consistency
        let mut filtered_values = [0.0; 6];
        
        // Get time delta for filtering
        let dt = if self.last_reading_time > 0 {
            ((current_time - self.last_reading_time) as f32) / 1_000_000.0 // Convert µs to seconds
        } else {
            0.01 // Default to 10ms if first reading
        };
        self.last_reading_time = current_time;
        
        // Update sample frequency for AHRS algorithm if needed
        if dt > 0.0 {
            self.sample_freq = 1.0 / dt;
        }
        
        // Constants for filtering
        const MEASUREMENT_NOISE_ACCEL: f32 = 0.03;
        const MEASUREMENT_NOISE_GYRO: f32 = 0.02;
        const PROCESS_NOISE_ACCEL: f32 = 0.005;
        const PROCESS_NOISE_GYRO: f32 = 0.003;
        
        // Apply Kalman filtering to each axis
        for i in 0..6 {
            let (estimate, error_estimate) = DriftCorrection::kalman_filter(
                raw_values[i],
                self.filtered_values[i],
                self.kalman_error_estimates[i],
                if i < 3 { MEASUREMENT_NOISE_ACCEL } else { MEASUREMENT_NOISE_GYRO },
                if i < 3 { PROCESS_NOISE_ACCEL } else { PROCESS_NOISE_GYRO }
            );
            
            filtered_values[i] = estimate;
            self.kalman_error_estimates[i] = error_estimate;
        }
        
        // Detect if device is stationary for bias correction
        let accel_samples = [filtered_values[0], filtered_values[1], filtered_values[2]];
        let gyro_samples = [filtered_values[3], filtered_values[4], filtered_values[5]];
        let is_stationary = DriftCorrection::is_stationary(&accel_samples, &gyro_samples, 0.02);
        
        // Apply adaptive bias estimation to gyroscope values
        for i in 0..3 {
            let (corrected_value, new_bias) = DriftCorrection::adaptive_gyro_bias(
                filtered_values[i+3],
                self.gyro_bias[i],
                is_stationary,
                0.005 // Learning rate - small to avoid oscillations
            );
            
            filtered_values[i+3] = corrected_value;
            self.gyro_bias[i] = new_bias;
        }
        
        // Apply spike detection and filtering
        for i in 0..6 {
            filtered_values[i] = DriftCorrection::filter_spikes(
                filtered_values[i],
                self.filtered_values[i],
                if i < 3 { 0.5 } else { 5.0 }, // Max change threshold
                0.3 // Filter weight
            );
        }
        
        // Save filtered values for next iteration
        self.filtered_values = filtered_values;
        
        filtered_values
    }
    
    /// Enable advanced filtering for drift reduction
    /// 
    /// Activates:
    /// - Kalman filtering
    /// - Adaptive bias estimation
    /// - Spike detection
    pub fn enable_advanced_filtering(&mut self) {
        self.advanced_filtering = true;
    }
    
    /// Disable advanced filtering
    pub fn disable_advanced_filtering(&mut self) {
        self.advanced_filtering = false;
    }
    
    /// Get current gyroscope bias values
    pub fn get_gyro_bias(&self) -> [f32; 3] {
        self.gyro_bias
    }
    
    /// Manually set gyroscope bias values
    pub fn set_gyro_bias(&mut self, bias: [f32; 3]) {
        self.gyro_bias = bias;
    }
    
    /// Reset all filtering state
    /// Useful after recalibration or when detection major discontinuity
    pub fn reset_filter_state(&mut self) {
        self.filtered_values = [0.0; 6];
        self.kalman_error_estimates = [0.0; 6];
        self.last_reading_time = 0;
    }
}
