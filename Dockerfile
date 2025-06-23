# Use a standard Rust image as base
FROM rust:latest

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    wget \
    flex \
    bison \
    gperf \
    python3 \
    python3-pip \
    python3-venv \
    cmake \
    ninja-build \
    ccache \
    libffi-dev \
    libssl-dev \
    dfu-util \
    libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Install espup for Espressif Rust toolchain
RUN cargo install espup

# Install Espressif Rust toolchain and tools
RUN espup install

# Set working directory
WORKDIR /workspace

# Copy project files
COPY . .

# Source the ESP environment and build the project
RUN /bin/bash -c "source $HOME/export-esp.sh && cargo build --target xtensa-esp32-none-elf"

# Keep container running for interactive use
CMD ["bash"]
