#!/bin/bash

# Build script for ESP32 project using Docker

echo "Building ESP32 firmware using Docker container..."

# Build the Docker image
docker build -t esp32-self-balancing-robot .

# Run the container and build the project
docker run --rm -v "$(pwd):/workspace" -w /workspace esp32-self-balancing-robot cargo build --target xtensa-esp32-none-elf

echo "Build complete! Firmware binary should be available at: target/xtensa-esp32-none-elf/debug/self-balancing-robot2"
