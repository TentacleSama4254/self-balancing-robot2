# Self-Balancing Robot ESP32 Project

This project is designed for ESP32 microcontrollers and implements a self-balancing robot with IMU sensors.

## Prerequisites

Since this project uses ESP32 Rust development, you'll need to set up the build environment. Due to ARM64 Windows limitations with the ESP toolchain, we recommend using Docker for building.

### Option 1: Using Docker (Recommended for ARM64 Windows)

1. **Install Docker Desktop for Windows:**
   - Download from: https://www.docker.com/products/docker-desktop/
   - Follow the installation instructions
   - Make sure Docker is running

2. **Build the project:**
   ```powershell
   # Navigate to project directory
   cd "c:\Users\inush\Code - Win\self-balancing-robot2"
   
   # Run the build script
   .\build.ps1
   ```

   Or manually:
   ```powershell
   # Build Docker image
   docker build -t esp32-self-balancing-robot .
   
   # Build the firmware
   docker run --rm -v "${PWD}:/workspace" -w /workspace esp32-self-balancing-robot cargo build --target xtensa-esp32-none-elf
   ```

3. **After building:**
   - The firmware binary will be available at: `target/xtensa-esp32-none-elf/debug/self-balancing-robot2`
   - You can now run the project in Wokwi

### Option 2: Using Docker Compose

```powershell
# Build and run
docker-compose up --build

# For interactive development
docker-compose run esp32-builder bash
```

### Option 3: Manual ESP Toolchain Installation (x64 Windows only)

If you're on x64 Windows (not ARM64), you can install the toolchain directly:

```powershell
# Install espup
cargo install espup

# Install ESP toolchain
espup install

# Source the environment (in each new terminal)
# This step varies by shell - follow espup instructions

# Build the project
cargo build --target xtensa-esp32-none-elf
```

## Project Structure

- `src/bin/main.rs` - Main application entry point
- `src/imu/` - IMU sensor drivers and calibration
- `src/i2c_wrapper.rs` - I2C communication wrapper
- `wokwi.toml` - Wokwi simulator configuration
- `Cargo.toml` - Rust project configuration

## Running in Wokwi

1. Make sure the firmware is built (binary exists at the path specified in `wokwi.toml`)
2. Open the project in Wokwi
3. The simulator will load the firmware automatically

## Hardware Components

- ESP32 microcontroller
- ADXL345 accelerometer (I2C address: typically 0x53)
- ITG3200 gyroscope (I2C address: typically 0x68)
- I2C pins: SDA=GPIO33, SCL=GPIO25

## Development

To modify the code:
1. Edit the source files
2. Rebuild using Docker: `.\build.ps1`
3. Test in Wokwi or deploy to hardware

## Troubleshooting

- **"firmware binary not found"**: Make sure to build the project first using the Docker method
- **Docker issues**: Ensure Docker Desktop is running and you have sufficient permissions
- **Build errors**: Check that all dependencies are properly specified in Cargo.toml
