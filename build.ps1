# Build script for ESP32 project using Docker (PowerShell)

Write-Host "Building ESP32 firmware using Docker container..." -ForegroundColor Green

# Build the Docker image
Write-Host "Building Docker image..." -ForegroundColor Yellow
docker build -t esp32-self-balancing-robot .

if ($LASTEXITCODE -ne 0) {
    Write-Host "Docker build failed!" -ForegroundColor Red
    exit 1
}

# Run the container and build the project
Write-Host "Building firmware..." -ForegroundColor Yellow
docker run --rm -v "${PWD}:/workspace" -w /workspace esp32-self-balancing-robot cargo build --target xtensa-esp32-none-elf

if ($LASTEXITCODE -eq 0) {
    Write-Host "Build complete! Firmware binary should be available at: target/xtensa-esp32-none-elf/debug/self-balancing-robot2" -ForegroundColor Green
} else {
    Write-Host "Build failed!" -ForegroundColor Red
}
