// Constants for ADXL345 registers
// Based on the original C++ implementation from FreeSixIMU

// ADXL345 Register Addresses
pub const DEVID: u8 = 0x00;        // Device ID
pub const THRESH_TAP: u8 = 0x1D;   // Tap threshold
pub const OFSX: u8 = 0x1E;         // X-axis offset
pub const OFSY: u8 = 0x1F;         // Y-axis offset
pub const OFSZ: u8 = 0x20;         // Z-axis offset
pub const DUR: u8 = 0x21;          // Tap duration
pub const LATENT: u8 = 0x22;       // Tap latency
pub const WINDOW: u8 = 0x23;       // Tap window
pub const THRESH_ACT: u8 = 0x24;   // Activity threshold
pub const THRESH_INACT: u8 = 0x25; // Inactivity threshold
pub const TIME_INACT: u8 = 0x26;   // Inactivity time
pub const ACT_INACT_CTL: u8 = 0x27; // Axis enable control for activity/inactivity
pub const THRESH_FF: u8 = 0x28;    // Free-fall threshold
pub const TIME_FF: u8 = 0x29;      // Free-fall time
pub const TAP_AXES: u8 = 0x2A;     // Axis control for tap/double tap
pub const ACT_TAP_STATUS: u8 = 0x2B; // Source of tap/double tap
pub const BW_RATE: u8 = 0x2C;      // Data rate and power mode control
pub const POWER_CTL: u8 = 0x2D;    // Power control
pub const INT_ENABLE: u8 = 0x2E;   // Interrupt enable control
pub const INT_MAP: u8 = 0x2F;      // Interrupt mapping control
pub const INT_SOURCE: u8 = 0x30;   // Interrupt source
pub const DATA_FORMAT: u8 = 0x31;  // Data format control
pub const DATAX0: u8 = 0x32;       // X-axis data 0
pub const DATAX1: u8 = 0x33;       // X-axis data 1
pub const DATAY0: u8 = 0x34;       // Y-axis data 0
pub const DATAY1: u8 = 0x35;       // Y-axis data 1
pub const DATAZ0: u8 = 0x36;       // Z-axis data 0
pub const DATAZ1: u8 = 0x37;       // Z-axis data 1
pub const FIFO_CTL: u8 = 0x38;     // FIFO control
pub const FIFO_STATUS: u8 = 0x39;  // FIFO status

// Power Control Register Bits
pub const PCTL_MEASURE: u8 = 0x08; // Measurement mode
pub const PCTL_SLEEP: u8 = 0x04;   // Sleep mode
pub const PCTL_STANDBY: u8 = 0x00; // Standby mode

// Data Format Register Bits
pub const SELF_TEST: u8 = 0x80;    // Self-test enable
pub const SPI: u8 = 0x40;          // SPI mode (3/4 wire)
pub const INT_INVERT: u8 = 0x20;   // Interrupt inversion
pub const FULL_RES: u8 = 0x08;     // Full resolution mode
pub const JUSTIFY: u8 = 0x04;      // Data justification

// Range settings
pub const RANGE_2G: u8 = 0x00;     // ±2g range
pub const RANGE_4G: u8 = 0x01;     // ±4g range
pub const RANGE_8G: u8 = 0x02;     // ±8g range
pub const RANGE_16G: u8 = 0x03;    // ±16g range

// Default ADXL345 address
pub const ADXL345_ADDR: u8 = 0x53; // When SDO/ALT ADDRESS pin is low
