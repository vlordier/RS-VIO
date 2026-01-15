# RS-VIO Docker Image (runtime binaries: run_euroc, run_4seasons, run_tum)

FROM rustlang/rust:nightly-slim AS builder

# Install system dependencies for compilation
RUN apt-get update && apt-get install -y --no-install-recommends \
    pkg-config \
    libssl-dev \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Cache dependencies
COPY Cargo.toml Cargo.lock ./
RUN mkdir src && echo "fn main() {}" > src/main.rs
RUN cargo fetch --locked
RUN rm -rf src

# Copy full workspace
COPY src ./src
COPY config ./config
COPY scripts ./scripts

# Build the runtime binaries (no default features for embedded-friendly runtime)
RUN cargo build --locked --release \
    --bin run_euroc \
    --bin run_4seasons \
    --bin run_tum

FROM debian:trixie-slim AS runtime

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN useradd -r -s /bin/false rs-vio

WORKDIR /app

# Copy binaries and configs
COPY --from=builder /app/target/release/run_euroc /usr/local/bin/
COPY --from=builder /app/target/release/run_4seasons /usr/local/bin/
COPY --from=builder /app/target/release/run_tum /usr/local/bin/
COPY --from=builder /app/config ./config
COPY --from=builder /app/scripts ./scripts

RUN chown -R rs-vio:rs-vio /usr/local/bin/run_* /app && \
    chmod 755 /usr/local/bin/run_*

USER rs-vio

# Default to Euroc runner; override with CMD/entrypoint as needed
ENTRYPOINT ["/usr/local/bin/run_euroc"]