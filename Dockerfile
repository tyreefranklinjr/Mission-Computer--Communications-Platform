# Dockerfile
#
# Multi-stage build for the C++/C mission computer simulation.
# This builds and runs the software simulation only; it does not, and
# cannot, contain or emulate real ESP32-C3 / CAN hardware. See README.md.
#
# NOTE: this Dockerfile has been reviewed carefully by hand but has not
# been built against a live Docker daemon in the environment this project
# was developed in. See README.md's "Simulation notes" section.

# ---- build stage ----------------------------------------------------------
FROM ubuntu:22.04 AS build

RUN apt-get update && apt-get install -y --no-install-recommends \
        cmake \
        ninja-build \
        g++ \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY CMakeLists.txt ./
COPY src ./src
COPY tests ./tests

RUN cmake -G Ninja -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build

# ---- runtime stage ---------------------------------------------------------
FROM ubuntu:22.04 AS runtime

# The runtime image only needs the C++ standard library, not the compiler
# toolchain used to build it.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libstdc++6 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY --from=build /src/build/mission_computer ./mission_computer

# Default to the live sim mode; override the command to run fault-sweep
# instead, e.g.:
#   docker run <image> ./mission_computer fault-sweep 500
ENTRYPOINT ["./mission_computer"]
CMD ["sim"]
