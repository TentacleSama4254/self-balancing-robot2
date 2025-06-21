// Constants for ITG3200 registers
// Based on the original C++ implementation from FreeSixIMU

// Register addresses
pub const WHO_AM_I: u8 = 0x00;
pub const SMPLRT_DIV: u8 = 0x15;
pub const DLPF_FS: u8 = 0x16;
pub const INT_CFG: u8 = 0x17;
pub const INT_STATUS: u8 = 0x1A;
pub const TEMP_OUT: u8 = 0x1B;
pub const GYRO_XOUT: u8 = 0x1D;
pub const GYRO_YOUT: u8 = 0x1F;
pub const GYRO_ZOUT: u8 = 0x21;
pub const PWR_MGM: u8 = 0x3E;

// Sample Rate Divider
pub const NOSRDIVIDER: u8 = 0; // default no sample rate divider

// Full-Scale Range
pub const RANGE2000: u8 = 3; // default full-scale range of 2000°/sec

// Digital Low-Pass Filter (DLPF) and Sample Rate Configuration
pub const BW256_SR8: u8 = 0; // 256Hz bandwidth, 8kHz sample rate
pub const BW188_SR1: u8 = 1;
pub const BW098_SR1: u8 = 2;
pub const BW042_SR1: u8 = 3;
pub const BW020_SR1: u8 = 4;
pub const BW010_SR1: u8 = 5;
pub const BW005_SR1: u8 = 6;

// Clock Source
pub const INTERNALOSC: u8 = 0; // Internal oscillator
pub const PLL_XGYRO_REF: u8 = 1; // PLL with X Gyro reference
pub const PLL_YGYRO_REF: u8 = 2; // PLL with Y Gyro reference
pub const PLL_ZGYRO_REF: u8 = 3; // PLL with Z Gyro reference
pub const PLL_EXTERNAL32: u8 = 4; // PLL with external 32.768kHz reference
pub const PLL_EXTERNAL19: u8 = 5; // PLL with external 19.2MHz reference

// Power Management
pub const PWRMGM_HRESET: u8 = 0x80; // Hard reset
pub const PWRMGM_SLEEP: u8 = 0x40; // Sleep mode
pub const PWRMGM_STBY_XG: u8 = 0x20; // X Gyro standby
pub const PWRMGM_STBY_YG: u8 = 0x10; // Y Gyro standby
pub const PWRMGM_STBY_ZG: u8 = 0x08; // Z Gyro standby

// Digital Low-Pass Filter and Full-Scale registers
pub const DLPFFS_FS_SEL: u8 = 0x18; // Full-Scale selection mask
pub const DLPFFS_DLPF_CFG: u8 = 0x07; // DLPF configuration mask

// Interrupt Configuration
pub const INTCFG_ACTL: u8 = 0x80; // Logic level for INT output pin (active low)
pub const INTCFG_OPEN: u8 = 0x40; // Drive type for INT output pin (open drain)
pub const INTCFG_LATCH_INT_EN: u8 = 0x20; // Latch mode (latch until cleared)
pub const INTCFG_INT_ANYRD_2CLEAR: u8 = 0x10; // Clear INT on any register read
pub const INTCFG_ITG_RDY_EN: u8 = 0x04; // Enable interrupt when device is ready
pub const INTCFG_RAW_RDY_EN: u8 = 0x01; // Enable interrupt when data is ready

// Interrupt Status
pub const INTSTATUS_ITG_RDY: u8 = 0x04; // Device ready interrupt
pub const INTSTATUS_RAW_DATA_RDY: u8 = 0x01; // Raw data ready interrupt

// Device startup delay
pub const GYROSTART_UP_DELAY: u32 = 70; // ms

// ITG3200 default address
pub const ITG3200_ADDR: u8 = 0x68;
