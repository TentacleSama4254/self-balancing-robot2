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
        }
        }
    
    /// Initialize both sensors
    pub fn init<D>(&mut self, delay_fn: &mut D) -> Result<(), E>
    where
        D: FnMut(u32),
    {
        self.init_with_settings(FIMU_ACC_ADDR, FIMU_ITG3200_DEF_ADDR, false, delay_fn)
    }
    
    /// Initialize both sensors with fast mode option
    pub fn init_with_fast_mode<D>(&mut self, fast_mode: bool, delay_fn: &mut D) -> Result<(), E>
    where 
        D: FnMut(u32),
    {
        self.init_with_settings(FIMU_ACC_ADDR, FIMU_ITG3200_DEF_ADDR, fast_mode, delay_fn)
    }
    
    /// Initialize with custom settings
    pub fn init_with_settings<D>(
        &mut self, 
        acc_addr: u8, 
        gyro_addr: u8, 
        _fast_mode: bool,
        delay_fn: &mut D
    ) -> Result<(), E>
    where
        D: FnMut(u32),
    {
        // Initialize accelerometer
        self.acc = ADXL345::new_with_address(self.i2c.clone(), acc_addr);
        self.acc.init()?;
        
        // Initialize gyroscope
        self.gyro = ITG3200::new_with_address(self.i2c.clone(), gyro_addr);
        self.gyro.init()?;
        self.gyro = ITG3200::new_with_address(self.i2c.clone(), gyro_addr);
        self.gyro.init()?;
        
        // Wait for gyro to stabilize
        delay_fn(1000);
        
        // Calibrate the ITG3200
        self.zero_calibrate(128, delay_fn)?;
        
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
        // Calibrate gyroscope
        self.zero_calibrate(gyro_samples, delay_fn)?;
        
        // Calibrate accelerometer - find offsets when device is flat
        self.calibrate_accelerometer(accel_samples, delay_fn)?;
        
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
        
        for _ in 0..samples {
            let (x, y, z) = self.acc.read_accel_g()?;
            x_sum += x;
            y_sum += y;
            z_sum += z;
            
            // Small delay between samples
            delay_fn(5); // 5ms delay
        }
        
        // Calculate average values
        let x_avg = x_sum / samples as f32;
        let y_avg = y_sum / samples as f32;
        let z_avg = z_sum / samples as f32;
        
        // Compute gains to normalize readings (X, Y should be 0, Z should be 1G)
        // Only apply gains if readings are not too far from expected
        if x_avg.abs() < 0.3 && y_avg.abs() < 0.3 && (z_avg - 1.0).abs() < 0.3 {
            // Compute inverse gains to normalize accelerometer readings
            let x_gain = if x_avg.abs() > 0.01 { 0.0 / x_avg } else { 1.0 };
            let y_gain = if y_avg.abs() > 0.01 { 0.0 / y_avg } else { 1.0 };
            let z_gain = if z_avg.abs() > 0.01 { 1.0 / z_avg } else { 1.0 };
            
            self.acc.set_axis_gains(x_gain, y_gain, z_gain);
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
    pub fn get_values(&mut self) -> Result<[f32; 6], E> {
        let (acc_x, acc_y, acc_z) = self.acc.read_accel_g()?;
        let (gyro_x, gyro_y, gyro_z) = self.gyro.read_gyro()?;
        
        Ok([acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z])
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
    
    /// Fast inverse square-root
    /// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
    fn inv_sqrt(&self, x: f32) -> f32 {
        // Using standard library to maintain precision
        1.0 / sqrtf(x)
    }
}
