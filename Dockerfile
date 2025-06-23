# Use the Espressif Rust image for ESP32
FROM espressif/idf-rust:esp32_latest

# Set working directory
WORKDIR /workspace

# Copy project files
COPY . .

# Build the project
RUN cargo build --target xtensa-esp32-none-elf

# Keep container running for interactive use
CMD ["bash"]
