use embedded_hal::i2c::I2c;

use crate::imu::adxl345_registers::*;

/// ADXL345 accelerometer driver
pub struct ADXL345<I2C> {
    i2c: I2C,
    address: u8,
    gains: [f32; 3], // X, Y, Z gains
}

impl<I2C, E> ADXL345<I2C>
where
    I2C: I2c<Error = E>,
{
    /// Create a new ADXL345 driver with default address
    pub fn new(i2c: I2C) -> Self {
        Self::new_with_address(i2c, ADXL345_ADDR)
    }

    /// Create a new ADXL345 driver with custom address
    pub fn new_with_address(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            gains: [1.0, 1.0, 1.0],
        }
    }

    /// Initialize the accelerometer
    pub fn init(&mut self) -> Result<(), E> {
        // Put the ADXL345 into +/- 4G range by default
        self.set_range_setting(RANGE_4G)?;
        
        // Put the ADXL345 into Measurement Mode
        self.power_on()?;
        
        Ok(())
    }

    /// Power on the device (put in measurement mode)
    pub fn power_on(&mut self) -> Result<(), E> {
        self.write_register(POWER_CTL, PCTL_MEASURE)
    }

    /// Read accelerometer data (X, Y, Z)
    pub fn read_accel(&mut self) -> Result<(i16, i16, i16), E> {
        let mut buffer = [0u8; 6];
        self.read_registers(DATAX0, &mut buffer)?;
        
        // Convert the data
        let x = ((buffer[1] as i16) << 8) | (buffer[0] as i16);
        let y = ((buffer[3] as i16) << 8) | (buffer[2] as i16);
        let z = ((buffer[5] as i16) << 8) | (buffer[4] as i16);
        
        Ok((x, y, z))
    }

    /// Read accelerometer data in g units
    pub fn read_accel_g(&mut self) -> Result<(f32, f32, f32), E> {
        let (x, y, z) = self.read_accel()?;
        
        // Default scale factor is ~3.9 mg/LSB for +/-2g range
        // This will need to be adjusted based on the range setting
        let scale_factor = match self.get_range_setting()? {
            RANGE_2G => 3.9,
            RANGE_4G => 7.8,
            RANGE_8G => 15.6,
            RANGE_16G => 31.2,
            _ => 3.9, // Default to 2G if unknown
        };
        
        let x_g = (x as f32) * scale_factor / 1000.0 * self.gains[0];
        let y_g = (y as f32) * scale_factor / 1000.0 * self.gains[1];
        let z_g = (z as f32) * scale_factor / 1000.0 * self.gains[2];
        
        Ok((x_g, y_g, z_g))
    }

    /// Set the range setting
    pub fn set_range_setting(&mut self, range: u8) -> Result<(), E> {
        if range > RANGE_16G {
            // Invalid range setting
            return self.write_register(DATA_FORMAT, RANGE_2G); // Default to 2G
        }
        
        // Keep existing settings except for range
        let mut format = self.read_register(DATA_FORMAT)?;
        format = (format & 0xFC) | (range & 0x03); // Clear and set range bits (0-1)
        
        self.write_register(DATA_FORMAT, format)
    }

    /// Get the current range setting
    pub fn get_range_setting(&mut self) -> Result<u8, E> {
        let format = self.read_register(DATA_FORMAT)?;
        Ok(format & 0x03) // Extract range bits (0-1)
    }

    /// Set the full resolution bit
    pub fn set_full_res_bit(&mut self, enabled: bool) -> Result<(), E> {
        let mut format = self.read_register(DATA_FORMAT)?;
        
        format = if enabled {
            format | FULL_RES
        } else {
            format & !FULL_RES
        };
        
        self.write_register(DATA_FORMAT, format)
    }

    /// Set the axis gains
    pub fn set_axis_gains(&mut self, x_gain: f32, y_gain: f32, z_gain: f32) {
        self.gains = [x_gain, y_gain, z_gain];
    }

    /// Get the axis gains
    pub fn get_axis_gains(&self) -> [f32; 3] {
        self.gains
    }

    /// Set offsets for each axis
    pub fn set_axis_offset(&mut self, x_offset: i8, y_offset: i8, z_offset: i8) -> Result<(), E> {
        self.write_register(OFSX, x_offset as u8)?;
        self.write_register(OFSY, y_offset as u8)?;
        self.write_register(OFSZ, z_offset as u8)
    }

    /// Get offsets for each axis
    pub fn get_axis_offset(&mut self) -> Result<(i8, i8, i8), E> {
        let x = self.read_register(OFSX)? as i8;
        let y = self.read_register(OFSY)? as i8;
        let z = self.read_register(OFSZ)? as i8;
        
        Ok((x, y, z))
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
