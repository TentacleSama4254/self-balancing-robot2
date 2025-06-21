#![cfg(test)]
#![no_std]

use self_balancing_robot2::imu::drift_correction::DriftCorrection;

#[test]
fn test_kalman_filter() {
    // Test with noisy data
    let mut prev_estimate = 0.0;
    let mut error_estimate = 1.0;
    let measurement_noise = 0.1;
    let process_noise = 0.01;
    
    // Simulate constant value with noise
    let true_value = 5.0;
    let noisy_measurements = [
        true_value + 0.5,  // 5.5
        true_value - 0.3,  // 4.7
        true_value + 0.2,  // 5.2
        true_value - 0.1,  // 4.9
        true_value + 0.4,  // 5.4
    ];
    
    // Apply Kalman filter to each measurement
    let mut filtered_values = Vec::new();
    for &measurement in &noisy_measurements {
        let (estimate, new_error) = DriftCorrection::kalman_filter(
            measurement,
            prev_estimate,
            error_estimate,
            measurement_noise,
            process_noise
        );
        
        filtered_values.push(estimate);
        prev_estimate = estimate;
        error_estimate = new_error;
    }
    
    // Check that filtered values converge to true value
    assert!((filtered_values.last().unwrap() - true_value).abs() < 0.3);
    
    // Check that error estimate decreases over time
    assert!(error_estimate < 1.0);
}

#[test]
fn test_complementary_filter() {
    // Test complementary filter with sample data
    let accel_angle = 10.0; // Angle from accelerometer
    let gyro_rate = 2.0;    // Angular rate from gyroscope
    let prev_angle = 8.0;   // Previous filtered angle
    let dt = 0.1;           // Time step in seconds
    let alpha = 0.98;       // Weight for gyroscope data
    
    let filtered_angle = DriftCorrection::complementary_filter(
        accel_angle, 
        gyro_rate, 
        prev_angle, 
        dt, 
        alpha
    );
    
    // Expected result: alpha * (prev_angle + gyro_rate * dt) + (1 - alpha) * accel_angle
    let expected = alpha * (prev_angle + gyro_rate * dt) + (1.0 - alpha) * accel_angle;
    
    assert_eq!(filtered_angle, expected);
}

#[test]
fn test_adaptive_gyro_bias() {
    // Initial conditions
    let gyro_reading = 0.2; // Small non-zero value when stationary
    let current_bias = 0.0;
    let is_stationary = true;
    let learning_rate = 0.1;
    
    // Apply adaptive bias estimation when stationary
    let (corrected_reading, new_bias) = DriftCorrection::adaptive_gyro_bias(
        gyro_reading,
        current_bias,
        is_stationary,
        learning_rate
    );
    
    // New bias should move toward the gyro reading
    assert!(new_bias > 0.0);
    assert!(new_bias <= gyro_reading * learning_rate);
    
    // Corrected reading should be closer to zero
    assert!(corrected_reading.abs() < gyro_reading.abs());
    
    // Test with moving device
    let gyro_reading_moving = 5.0;
    let (corrected_reading_moving, new_bias_moving) = DriftCorrection::adaptive_gyro_bias(
        gyro_reading_moving,
        new_bias, // Use new bias from previous test
        false,    // Not stationary
        learning_rate
    );
    
    // Bias should not change when not stationary
    assert_eq!(new_bias_moving, new_bias);
    
    // Reading should be corrected by the bias amount
    assert_eq!(corrected_reading_moving, gyro_reading_moving - new_bias);
}

#[test]
fn test_is_stationary() {
    // Test with very stable readings
    let stable_accel = [0.0, 0.0, 1.0]; // Gravity aligned with Z-axis
    let stable_gyro = [0.0, 0.0, 0.0];  // No rotation
    
    let result_stable = DriftCorrection::is_stationary(&stable_accel, &stable_gyro, 0.1);
    assert!(result_stable);
    
    // Test with movement
    let moving_gyro = [0.0, 0.2, 0.0]; // Small rotation around Y-axis
    let result_moving = DriftCorrection::is_stationary(&stable_accel, &moving_gyro, 0.1);
    assert!(!result_moving);
    
    // Test with accelerometer jitter
    let unstable_accel = [0.1, 0.1, 1.1]; // Slight movement
    let result_accel_unstable = DriftCorrection::is_stationary(&unstable_accel, &stable_gyro, 0.1);
    assert!(!result_accel_unstable);
}

#[test]
fn test_filter_spikes() {
    // Test with normal change
    let prev_value = 10.0;
    let current_value = 10.5; // Small, acceptable change
    let max_change = 1.0;
    let filter_alpha = 0.3;
    
    let filtered_normal = DriftCorrection::filter_spikes(
        current_value,
        prev_value,
        max_change,
        filter_alpha
    );
    
    // Small change should pass through with no heavy filtering
    assert_eq!(filtered_normal, current_value);
    
    // Test with spike
    let spike_value = 15.0; // Large, sudden change
    
    let filtered_spike = DriftCorrection::filter_spikes(
        spike_value,
        prev_value,
        max_change,
        filter_alpha
    );
    
    // Spike should be heavily filtered
    assert!(filtered_spike > prev_value); // Should move toward spike
    assert!(filtered_spike < spike_value); // But not reach it fully
    // Specifically, it should be: prev_value + filter_alpha * (spike_value - prev_value)
    let expected = prev_value + filter_alpha * (spike_value - prev_value);
    assert_eq!(filtered_spike, expected);
}
