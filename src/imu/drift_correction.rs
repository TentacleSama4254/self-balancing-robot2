use libm::sqrtf;

/// Advanced drift correction algorithms for IMU data
pub struct DriftCorrection;

impl DriftCorrection {
    /// Applies a Kalman filter to reduce noise in sensor readings
    /// 
    /// This is a simplified single-variable Kalman filter implementation for each axis.
    /// 
    /// # Arguments
    /// * `measurement` - The current measurement
    /// * `prev_estimate` - The previous state estimate
    /// * `error_estimate` - The current error estimate
    /// * `measurement_noise` - The measurement noise (constant)
    /// * `error_process` - The process noise (constant)
    /// 
    /// # Returns
    /// A tuple containing the new estimate and the new error estimate
    pub fn kalman_filter(
        measurement: f32,
        prev_estimate: f32,
        error_estimate: f32,
        measurement_noise: f32,
        error_process: f32,
    ) -> (f32, f32) {
        // Prediction update
        let error_estimate_updated = error_estimate + error_process;
        
        // Measurement update
        let kalman_gain = error_estimate_updated / (error_estimate_updated + measurement_noise);
        let current_estimate = prev_estimate + kalman_gain * (measurement - prev_estimate);
        let new_error_estimate = (1.0 - kalman_gain) * error_estimate_updated;
        
        (current_estimate, new_error_estimate)
    }
    
    /// Applies a complementary filter to fuse gyroscope and accelerometer data
    /// 
    /// # Arguments
    /// * `accel_angle` - Angle derived from accelerometer
    /// * `gyro_rate` - Angular rate from gyroscope
    /// * `prev_angle` - Previous filtered angle
    /// * `dt` - Time step in seconds
    /// * `alpha` - Weight for gyroscope data (typically 0.98)
    /// 
    /// # Returns
    /// The filtered angle
    pub fn complementary_filter(
        accel_angle: f32,
        gyro_rate: f32,
        prev_angle: f32,
        dt: f32,
        alpha: f32,
    ) -> f32 {
        // Complementary filter: 
        // angle = alpha * (prev_angle + gyro_rate * dt) + (1 - alpha) * accel_angle
        alpha * (prev_angle + gyro_rate * dt) + (1.0 - alpha) * accel_angle
    }
    
    /// Adaptive Bias Estimation for gyroscope drift
    /// 
    /// # Arguments
    /// * `gyro_reading` - Current gyroscope reading
    /// * `current_bias` - Current estimated bias
    /// * `is_stationary` - Whether the system is determined to be stationary
    /// * `learning_rate` - Rate of bias correction (0.01-0.1)
    /// 
    /// # Returns
    /// Corrected gyroscope reading and updated bias estimate
    pub fn adaptive_gyro_bias(
        gyro_reading: f32,
        current_bias: f32,
        is_stationary: bool,
        learning_rate: f32,
    ) -> (f32, f32) {
        let mut new_bias = current_bias;
        
        // If the system is stationary, update bias estimate
        if is_stationary {
            new_bias = current_bias + learning_rate * (gyro_reading - current_bias);
        }
        
        // Return corrected reading and new bias
        (gyro_reading - new_bias, new_bias)
    }
    
    /// Detects if the device is stationary based on sensor variance
    /// 
    /// # Arguments
    /// * `accel_samples` - Recent accelerometer samples
    /// * `gyro_samples` - Recent gyroscope samples
    /// * `threshold` - Threshold for motion detection
    /// 
    /// # Returns
    /// Boolean indicating if device is stationary
    pub fn is_stationary(
        accel_samples: &[f32; 3],
        gyro_samples: &[f32; 3],
        threshold: f32,
    ) -> bool {
        // Calculate variance of gyro samples
        let gyro_variance = Self::calculate_variance(gyro_samples);
        
        // Calculate acceleration magnitude variance
        let acc_magnitude = sqrtf(
            accel_samples[0] * accel_samples[0] +
            accel_samples[1] * accel_samples[1] +
            accel_samples[2] * accel_samples[2]
        );
        let acc_variance = (acc_magnitude - 1.0).abs(); // Deviation from 1g
        
        // Check if variances are below threshold
        gyro_variance < threshold && acc_variance < threshold
    }
    
    /// Calculate variance of a sample set
    fn calculate_variance(samples: &[f32; 3]) -> f32 {
        let mean = (samples[0] + samples[1] + samples[2]) / 3.0;
        
        let variance_sum = 
            (samples[0] - mean) * (samples[0] - mean) + 
            (samples[1] - mean) * (samples[1] - mean) + 
            (samples[2] - mean) * (samples[2] - mean);
        
        variance_sum / 3.0
    }
    
    /// Apply spike detection and filtering to sensor readings
    /// 
    /// # Arguments
    /// * `current` - Current sensor reading
    /// * `previous` - Previous sensor reading
    /// * `max_change` - Maximum allowed change between readings
    /// * `filter_alpha` - Filter weight (0-1)
    /// 
    /// # Returns
    /// Filtered sensor reading
    pub fn filter_spikes(
        current: f32,
        previous: f32,
        max_change: f32,
        filter_alpha: f32,
    ) -> f32 {
        let change = (current - previous).abs();
        
        if change > max_change {
            // Spike detected, apply heavy filtering
            previous + filter_alpha * (current - previous)
        } else {
            // Normal reading
            current
        }
    }
}
