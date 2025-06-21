use embedded_hal::i2c::I2c;

use crate::imu::itg3200_registers::*;

/// ITG3200 gyroscope driver
pub struct ITG3200<I2C> {
    i2c: I2C,
    address: u8,
    x_gain: f32,
    y_gain: f32,
    z_gain: f32,
    x_offset: i16,
    y_offset: i16,
    z_offset: i16,
    x_rev: bool,
    y_rev: bool, 
    z_rev: bool,
}

impl<I2C, E> ITG3200<I2C>
where 
    I2C: I2c<Error = E>,
{
    /// Create a new ITG3200 driver with default address
    pub fn new(i2c: I2C) -> Self {
        Self::new_with_address(i2c, ITG3200_ADDR)
    }

    /// Create a new ITG3200 driver with custom address
    pub fn new_with_address(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            x_gain: 1.0,
            y_gain: 1.0,
            z_gain: 1.0,
            x_offset: 0,
            y_offset: 0,
            z_offset: 0,
            x_rev: false,
            y_rev: false,
            z_rev: false,
        }
    }

    /// Initialize the gyroscope with default settings
    pub fn init(&mut self) -> Result<(), E> {
        // Default initialization: fast sample rate - divisor = 0, filter = 0, clocksrc = PLL_XGYRO_REF
        self.init_with_settings(NOSRDIVIDER, RANGE2000, BW256_SR8, PLL_XGYRO_REF, true, true)
    }

    /// Initialize the gyroscope with custom settings
    pub fn init_with_settings(
        &mut self, 
        sample_rate_div: u8, 
        range: u8, 
        filter_bw: u8, 
        clock_src: u8,
        itg_ready: bool,
        raw_data_ready: bool
    ) -> Result<(), E> {
        // Set sample rate divider
        self.set_sample_rate_div(sample_rate_div)?;
        
        // Set full-scale range
        self.set_full_scale_range(range)?;
        
        // Set filter bandwidth
        self.set_filter_bandwidth(filter_bw)?;
        
        // Set clock source
        self.set_clock_source(clock_src)?;
        
        // Set interrupt configuration
        self.set_itg_ready(itg_ready)?;
        self.set_raw_data_ready(raw_data_ready)?;
        
        // Small delay to allow gyro to stabilize
        // In embedded context, we'd use a delay function provided by HAL
        // For now we'll assume caller will handle delay
        
        Ok(())
    }
    
    /// Set the sample rate divider
    pub fn set_sample_rate_div(&mut self, div: u8) -> Result<(), E> {
        self.write_register(SMPLRT_DIV, div)
    }

    /// Get the sample rate divider
    pub fn get_sample_rate_div(&mut self) -> Result<u8, E> {
        self.read_register(SMPLRT_DIV)
    }
    
    /// Set the full scale range (sensitivity)
    pub fn set_full_scale_range(&mut self, range: u8) -> Result<(), E> {
        let mut current = self.read_register(DLPF_FS)?;
        // Clear the FS_SEL bits and set them to the new value
        current = (current & !DLPFFS_FS_SEL) | ((range << 3) & DLPFFS_FS_SEL);
        self.write_register(DLPF_FS, current)
    }
    
    /// Get the current full scale range
    pub fn get_full_scale_range(&mut self) -> Result<u8, E> {
        let val = self.read_register(DLPF_FS)?;
        Ok((val & DLPFFS_FS_SEL) >> 3)
    }
    
    /// Set the filter bandwidth
    pub fn set_filter_bandwidth(&mut self, bw: u8) -> Result<(), E> {
        let mut current = self.read_register(DLPF_FS)?;
        // Clear the DLPF_CFG bits and set them to the new value
        current = (current & !DLPFFS_DLPF_CFG) | (bw & DLPFFS_DLPF_CFG);
        self.write_register(DLPF_FS, current)
    }
    
    /// Get the current filter bandwidth
    pub fn get_filter_bandwidth(&mut self) -> Result<u8, E> {
        let val = self.read_register(DLPF_FS)?;
        Ok(val & DLPFFS_DLPF_CFG)
    }
    
    /// Set the interrupt configuration (ITG ready)
    pub fn set_itg_ready(&mut self, state: bool) -> Result<(), E> {
        let mut current = self.read_register(INT_CFG)?;
        current = if state {
            current | INTCFG_ITG_RDY_EN
        } else {
            current & !INTCFG_ITG_RDY_EN
        };
        self.write_register(INT_CFG, current)
    }
    
    /// Set the interrupt configuration (raw data ready)
    pub fn set_raw_data_ready(&mut self, state: bool) -> Result<(), E> {
        let mut current = self.read_register(INT_CFG)?;
        current = if state {
            current | INTCFG_RAW_RDY_EN
        } else {
            current & !INTCFG_RAW_RDY_EN
        };
        self.write_register(INT_CFG, current)
    }
    
    /// Check if ITG is ready
    pub fn is_itg_ready(&mut self) -> Result<bool, E> {
        let status = self.read_register(INT_STATUS)?;
        Ok((status & INTSTATUS_ITG_RDY) != 0)
    }
    
    /// Check if raw data is ready
    pub fn is_raw_data_ready(&mut self) -> Result<bool, E> {
        let status = self.read_register(INT_STATUS)?;
        Ok((status & INTSTATUS_RAW_DATA_RDY) != 0)
    }
    
    /// Read the temperature sensor
    pub fn read_temp(&mut self) -> Result<f32, E> {
        let mut buffer = [0u8; 2];
        self.read_registers(TEMP_OUT, &mut buffer)?;
        
        // Combine the two bytes
        let raw_temp = ((buffer[0] as i16) << 8) | (buffer[1] as i16);
        
        // Convert to temperature in degrees Celsius
        // According to datasheet: 35°C + ((raw_value + 13200) / 280)
        let temp = 35.0 + ((raw_temp as f32 + 13200.0) / 280.0);
        
        Ok(temp)
    }
    
    /// Read raw gyro values
    pub fn read_gyro_raw(&mut self) -> Result<(i16, i16, i16), E> {
        let mut buffer = [0u8; 6];
        self.read_registers(GYRO_XOUT, &mut buffer)?;
        
        // Combine bytes for each axis
        let x = ((buffer[0] as i16) << 8) | (buffer[1] as i16);
        let y = ((buffer[2] as i16) << 8) | (buffer[3] as i16);
        let z = ((buffer[4] as i16) << 8) | (buffer[5] as i16);
        
        Ok((x, y, z))
    }
    
    /// Read calibrated gyro values (with offsets applied)
    pub fn read_gyro_raw_cal(&mut self) -> Result<(i16, i16, i16), E> {
        let (x, y, z) = self.read_gyro_raw()?;
        
        let x_cal = if self.x_rev { -(x - self.x_offset) } else { x - self.x_offset };
        let y_cal = if self.y_rev { -(y - self.y_offset) } else { y - self.y_offset };
        let z_cal = if self.z_rev { -(z - self.z_offset) } else { z - self.z_offset };
        
        Ok((x_cal, y_cal, z_cal))
    }
    
    /// Read gyro values in degrees per second
    pub fn read_gyro(&mut self) -> Result<(f32, f32, f32), E> {
        let (x, y, z) = self.read_gyro_raw_cal()?;
        
        // Convert to degrees per second (14.375 LSB per deg/sec for 2000 deg/sec range)
        let deg_per_sec_scale = 14.375;
        
        let x_dps = (x as f32) / deg_per_sec_scale * self.x_gain;
        let y_dps = (y as f32) / deg_per_sec_scale * self.y_gain;
        let z_dps = (z as f32) / deg_per_sec_scale * self.z_gain;
        
        Ok((x_dps, y_dps, z_dps))
    }
    
    /// Set gains for each axis
    pub fn set_gains(&mut self, x_gain: f32, y_gain: f32, z_gain: f32) {
        self.x_gain = x_gain;
        self.y_gain = y_gain;
        self.z_gain = z_gain;
    }
    
    /// Set offsets for each axis
    pub fn set_offsets(&mut self, x_offset: i16, y_offset: i16, z_offset: i16) {
        self.x_offset = x_offset;
        self.y_offset = y_offset;
        self.z_offset = z_offset;
    }
    
    /// Get current offsets for each axis
    pub fn get_offsets(&self) -> (i16, i16, i16) {
        (self.x_offset, self.y_offset, self.z_offset)
    }
    
    /// Set axis polarity reversal
    pub fn set_rev_polarity(&mut self, x_rev: bool, y_rev: bool, z_rev: bool) {
        self.x_rev = x_rev;
        self.y_rev = y_rev;
        self.z_rev = z_rev;
    }
    
    /// Perform zero calibration - should be called when gyro is stationary
    /// samples: number of samples to average
    pub fn zero_calibrate<D>(&mut self, samples: u16, delay_fn: &mut D) -> Result<(), E> 
    where
        D: FnMut(u32),
    {
        let mut x_sum: i32 = 0;
        let mut y_sum: i32 = 0;
        let mut z_sum: i32 = 0;
        
        for _ in 0..samples {
            let (x, y, z) = self.read_gyro_raw()?;
            x_sum += x as i32;
            y_sum += y as i32;
            z_sum += z as i32;
            
            // Small delay between samples
            delay_fn(5); // 5ms delay
        }
        
        // Calculate average offsets
        self.x_offset = (x_sum / samples as i32) as i16;
        self.y_offset = (y_sum / samples as i32) as i16;
        self.z_offset = (z_sum / samples as i32) as i16;
        
        Ok(())
    }
    
    /// Set the clock source
    pub fn set_clock_source(&mut self, source: u8) -> Result<(), E> {
        let mut current = self.read_register(PWR_MGM)?;
        // Clear the current clock source bits (bits 0-2) and set the new ones
        current = (current & 0xF8) | (source & 0x07);
        self.write_register(PWR_MGM, current)
    }
    
    /// Get the clock source
    pub fn get_clock_source(&mut self) -> Result<u8, E> {
        let val = self.read_register(PWR_MGM)?;
        Ok(val & 0x07)
    }
    
    /// Reset the device
    pub fn reset(&mut self) -> Result<(), E> {
        self.write_register(PWR_MGM, PWRMGM_HRESET)
    }
    
    /// Helper to write to a register
    fn write_register(&mut self, reg: u8, value: u8) -> Result<(), E> {
        self.i2c.write(self.address, &[reg, value])
    }
    
    /// Helper to read from a register
    fn read_register(&mut self, reg: u8) -> Result<u8, E> {
        let mut buffer = [0u8; 1];
        self.i2c.write_read(self.address, &[reg], &mut buffer)?;
        Ok(buffer[0])
    }
    
    /// Helper to read multiple registers
    fn read_registers(&mut self, reg: u8, buffer: &mut [u8]) -> Result<(), E> {
        self.i2c.write_read(self.address, &[reg], buffer)
    }
}
