use embedded_hal::i2c::I2c;
use crate::imu::adxl345_registers::{RANGE_2G, RANGE_4G, RANGE_8G, RANGE_16G};

// Enhanced calibration methods that can be used to improve IMU stability
pub struct ImprovedCalibration;

impl ImprovedCalibration {
    /// Calibrate accelerometer with enhanced algorithm to reduce drift
    /// This assumes the device is flat (X, Y near 0, Z near 1G)
    pub fn calibrate_accelerometer<I2C, D, E>(
        acc: &mut crate::imu::adxl345::ADXL345<I2C>,
        samples: u16,
        delay_fn: &mut D
    ) -> Result<(), E>
    where
        I2C: I2c<Error = E>,
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
            let (x, y, z) = acc.read_accel_g()?;
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
                
                acc.set_axis_gains(x_gain, y_gain, z_gain);
                
                // Also set hardware offsets if possible (convert to the range supported by hardware)
                // ADXL345 offsets are in raw LSB values, not g units
                let scale_factor = match acc.get_range_setting()? {
                    RANGE_2G => 3.9,
                    RANGE_4G => 7.8,
                    RANGE_8G => 15.6,
                    RANGE_16G => 31.2,
                    _ => 3.9, // Default to 2G if unknown
                };
                
                let x_offset = (-x_avg * 1000.0 / scale_factor).min(127.0).max(-128.0) as i8;
                let y_offset = (-y_avg * 1000.0 / scale_factor).min(127.0).max(-128.0) as i8;
                let z_offset = ((1.0 - z_avg) * 1000.0 / scale_factor).min(127.0).max(-128.0) as i8;
                
                acc.set_axis_offset(x_offset, y_offset, z_offset)?;
            }
        } else {
            // Good calibration data, apply full correction
            if x_avg.abs() < 0.3 && y_avg.abs() < 0.3 && (z_avg - 1.0).abs() < 0.3 {
                // First apply hardware offsets
                let scale_factor = match acc.get_range_setting()? {
                    RANGE_2G => 3.9,
                    RANGE_4G => 7.8,
                    RANGE_8G => 15.6,
                    RANGE_16G => 31.2,
                    _ => 3.9, // Default to 2G if unknown
                };
                
                let x_offset = (-x_avg * 1000.0 / scale_factor).min(127.0).max(-128.0) as i8;
                let y_offset = (-y_avg * 1000.0 / scale_factor).min(127.0).max(-128.0) as i8;
                let z_offset = ((1.0 - z_avg) * 1000.0 / scale_factor).min(127.0).max(-128.0) as i8;
                
                acc.set_axis_offset(x_offset, y_offset, z_offset)?;
                
                // Then apply fine-tuning gains - after reading new values with offsets applied
                // Small delay to allow settings to take effect
                delay_fn(10);
                
                // Re-read values after applying hardware offsets
                let mut new_x_sum: f32 = 0.0;
                let mut new_y_sum: f32 = 0.0;
                let mut new_z_sum: f32 = 0.0;
                
                for _ in 0..5 {
                    let (x, y, z) = acc.read_accel_g()?;
                    new_x_sum += x;
                    new_y_sum += y;
                    new_z_sum += z;
                    delay_fn(5);
                }
                
                let new_x_avg = new_x_sum / 5.0;
                let new_y_avg = new_y_sum / 5.0; 
                let new_z_avg = new_z_sum / 5.0;
                
                // Compute fine-tuning gains to get exact normalized accelerometer readings
                // Fix the division by zero issue and ensure proper normalization
                let x_gain = if new_x_avg.abs() > 0.01 { -new_x_avg.abs() / new_x_avg } else { 1.0 };
                let y_gain = if new_y_avg.abs() > 0.01 { -new_y_avg.abs() / new_y_avg } else { 1.0 };
                let z_gain = if new_z_avg.abs() > 0.1 { 1.0 / new_z_avg } else { 1.0 };
                
                acc.set_axis_gains(x_gain, y_gain, z_gain);
            }
        }
        
        Ok(())
    }
    
    /// Enhanced gyroscope calibration with additional drift compensation
    pub fn calibrate_gyro<I2C, D, E>(
        gyro: &mut crate::imu::itg3200::ITG3200<I2C>,
        samples: u16,
        delay_fn: &mut D
    ) -> Result<(), E>
    where
        I2C: I2c<Error = E>,
        D: FnMut(u32),
    {
        // First perform basic calibration to get offset values with more samples
        // This gives us a better baseline for the offsets
        gyro.zero_calibrate(samples, delay_fn)?;
        
        // We'll do a more refined calibration in multiple stages
        // Stage 1: Get initial readings after basic calibration
        let mut x_sum: f32 = 0.0;
        let mut y_sum: f32 = 0.0;
        let mut z_sum: f32 = 0.0;
        
        // Take more samples (30) for better accuracy
        for _ in 0..30 {
            let (x, y, z) = gyro.read_gyro()?;
            x_sum += x;
            y_sum += y;
            z_sum += z;
            delay_fn(10);
        }
        
        // Calculate average drift after first calibration
        let x_avg = x_sum / 30.0;
        let y_avg = y_sum / 30.0;
        let z_avg = z_sum / 30.0;
        
        // Stage 2: If there's still significant drift, directly adjust offsets further
        if x_avg.abs() > 0.05 || y_avg.abs() > 0.05 || z_avg.abs() > 0.05 {
            // Get current offsets
            let (curr_x_offset, curr_y_offset, curr_z_offset) = gyro.get_offsets();
            
            // Calculate additional offset correction
            // Convert from deg/s to raw values (14.375 LSB per deg/s)
            let lsb_per_dps = 14.375;
            let x_additional = (x_avg * lsb_per_dps) as i16;
            let y_additional = (y_avg * lsb_per_dps) as i16;
            let z_additional = (z_avg * lsb_per_dps) as i16;
            
            // Apply refined offsets
            gyro.set_offsets(
                curr_x_offset + x_additional,
                curr_y_offset + y_additional, 
                curr_z_offset + z_additional
            );
            
            // Short delay to let settings apply
            delay_fn(20);
            
            // Stage 3: Apply gain adjustments for any remaining drift
            let mut x2_sum: f32 = 0.0;
            let mut y2_sum: f32 = 0.0;
            let mut z2_sum: f32 = 0.0;
            
            // Take more samples to verify calibration
            for _ in 0..20 {
                let (x, y, z) = gyro.read_gyro()?;
                x2_sum += x;
                y2_sum += y;
                z2_sum += z;
                delay_fn(5);
            }
            
            let x2_avg = x2_sum / 20.0;
            let y2_avg = y2_sum / 20.0;
            let z2_avg = z2_sum / 20.0;
            
            // Apply fine-tuning gains to handle any remaining drift
            // This creates a slight counter-bias to zero out small movements
            let x_gain = if x2_avg.abs() > 0.02 { 1.0 - (x2_avg * 0.2) } else { 1.0 };
            let y_gain = if y2_avg.abs() > 0.02 { 1.0 - (y2_avg * 0.2) } else { 1.0 };
            let z_gain = if z2_avg.abs() > 0.02 { 1.0 - (z2_avg * 0.2) } else { 1.0 };
            
            gyro.set_gains(x_gain, y_gain, z_gain);
        }
        
        Ok(())
    }
}
