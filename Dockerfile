# Use a specific Rust version for stability
FROM rust:1.86.0

# Install system dependencies required for ESP development
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

# Install espup for managing ESP Rust toolchains
RUN cargo install espup --version 0.5.0

# Install the ESP Rust toolchain (esp-rs fork with xtensa support)
RUN espup install --toolchain esp --export-file "$HOME/export-esp.sh"

# Set the working directory
WORKDIR /workspace

# Copy project files
COPY . .

# Build the project using the ESP environment
RUN bash -c ". $HOME/export-esp.sh && cargo build -vv --target xtensa-esp32-none-elf"

# Open an interactive shell by default
CMD ["bash"]
