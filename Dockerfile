# RS-VIO Docker Image

# Use Rust official image for building
FROM rust:1.75-slim as builder

# Install system dependencies for compilation
RUN apt-get update && apt-get install -y \
    pkg-config \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy dependency files
COPY Cargo.toml Cargo.lock ./

# Create dummy src to cache dependencies
RUN mkdir src && echo "fn main() {}" > src/main.rs
RUN cargo build --release --target x86_64-unknown-linux-gnu
RUN rm -rf src

# Copy source code
COPY src ./src

# Build the application
RUN cargo build --release --target x86_64-unknown-linux-gnu

# Runtime stage
FROM debian:bookworm-slim

# Install runtime dependencies
RUN apt-get update && apt-get install -y \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user
RUN useradd -r -s /bin/false rs-vio

# Copy binary from builder
COPY --from=builder /app/target/x86_64-unknown-linux-gnu/release/rs-vio /usr/local/bin/

# Set ownership and permissions
RUN chown rs-vio:rs-vio /usr/local/bin/rs-vio && \
    chmod 755 /usr/local/bin/rs-vio

# Switch to non-root user
USER rs-vio

# Set working directory
WORKDIR /app

# Default command
CMD ["rs-vio"]