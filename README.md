# Resilient Autonomous Mission Computer and Degraded-Communications Platform

A software simulation of a distributed embedded autonomy system: sensor fusion, motion
planning, watchdog-driven mission safing, and a fault-tolerant communications layer with
automatic failover. Runs as ordinary Linux processes, the same way the author's companion
SDR project simulates IQ samples instead of driving a real radio. See
[Simulation notes](#simulation-notes) for exactly what is and is not real hardware.

**Stack:** C++20, C, Rust, FreeRTOS-style scheduling pattern, ESP32-C3 (target architecture only,
not present), CAN/TWAI (simulated), I2C/UART (as message-passing pattern), EKF, PID, A*, Docker, Linux

## Results (from real runs of this code)

| Metric | Result | How it was measured |
|---|---|---|
| Fault-sweep safe-state transition success | **100.00% (500 / 500)** | `mission_computer fault-sweep 500`, all 4 fault classes |
| comms_link failover latency | **22-41 ms** (target: < 250 ms), all 20 trials pass | `comms_link/run_failover_trials.sh`, 20 seeded runs |
| EKF vs. simulated true position error | typically < 0.3 m | `mission_computer sim`, logged every 50 ticks |
| Mission computer scheduler rate | 50 Hz (20 ms fixed tick) | `src/main.cpp` tick loop |
| Fault-sweep wall-clock runtime | ~0.07 s for 500 scenarios | measured on this machine |

Full per-scenario data is committed in `samples/`. `assets/dashboard.png` below was generated
from that data by `python/generate_report.py`.

![dashboard](assets/dashboard.png)

## How to build and run

### C++ / C mission computer

```bash
cmake -G Ninja -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build

./build/test_ekf
./build/test_astar

./build/mission_computer sim samples/ekf_trajectory.csv
./build/mission_computer fault-sweep 500 samples/fault_sweep_results.csv
```

### Rust comms_link

```bash
cd comms_link
cargo build --release
./target/release/comms_link       # one run, writes samples/comms_link_log.csv
./run_failover_trials.sh          # 20 seeded runs
```

### Dashboard

```bash
pip install -r python/requirements.txt
python3 python/generate_report.py
```

### Docker

```bash
docker build -t autonomy_platform .
docker run autonomy_platform sim
docker run autonomy_platform fault-sweep 500
```

## Architecture

Four simulated nodes, each stepped once per 20 ms tick by a fixed-rate scheduler in
`src/main.cpp`, communicating over a simulated CAN/TWAI bus:

- **Sensor node** generates simulated IMU/GNSS readings (`imu_gnss_sim.hpp`) and publishes them as CAN frames.
- **Navigation node** runs the EKF (`ekf.hpp`) to fuse IMU + GNSS into a position/velocity estimate, and A* (`astar.hpp`) to plan a path around an obstacle grid.
- **Control node** runs a PID controller (`pid.hpp`) to steer toward the next waypoint.
- **Supervisor node** watches heartbeats (`watchdog.hpp`) and drives the mission state machine (`mission_state.hpp`): `INIT -> CALIBRATING -> AUTONOMOUS`, dropping to `DEGRADED` on a comms outage or `SAFE_STOP` on an unrecovered fault.

The simulated CAN/TWAI bus (`can_bus.h`/`can_bus.c`) is a standalone C library: a fixed-size
ring buffer of `{id, priority, 8-byte payload}` frames, linked into C++ via `extern "C"`.

`fault_injector.hpp` deterministically generates fault scenarios (sensor dropout, CAN bus
fault, network outage, task hang) from a scenario number, so `fault-sweep` is fully
reproducible; `metrics.hpp` logs each scenario's expected vs. actual outcome to CSV.

`comms_link/` is a separate Rust binary demonstrating the fault-tolerant communications story
on its own: sequence tracking, ACK/retry, duplicate suppression, per-link success-rate scoring,
a `BinaryHeap` priority queue, and automatic failover during a simulated 30 s outage.

## Simulation notes

This runs entirely as ordinary processes on a regular Linux machine:

- No physical ESP32-C3 or CAN transceiver. `can_bus.c` is a RAM ring buffer, not a TWAI driver.
- FreeRTOS itself is not used. The "50 Hz scheduling" is a fixed-rate scheduler in plain C++, modeled on FreeRTOS's periodic task pattern.
- IMU/GNSS readings come from a scripted trajectory plus noise, not real sensors.
- The "30 s outage" in `comms_link` uses a virtual millisecond clock, not wall-clock time; the whole scenario finishes in under a second of real runtime, and the program says so in its own output.
- The Dockerfile was hand-reviewed but not built against a live Docker daemon (none was available in this environment).

This is an honest simulation of the architecture and behavior in the resume bullets above it,
built and measured on Linux, not a claim of code running on physical embedded hardware.

## Repository layout

```
src/            C++20 mission computer: EKF, PID, A*, watchdog, mission state machine,
                fault injector, plus can_bus.h/.c (the C piece)
tests/          test_ekf.cpp, test_astar.cpp
comms_link/     standalone Rust crate: fault-tolerant communications simulation
python/         generate_report.py, builds assets/dashboard.png from samples/*.csv
samples/        real CSV output from actual runs (not fabricated)
assets/         assets/dashboard.png, generated by python/generate_report.py
```

## Development notes

Only the files under `tests/` and `python/` were AI-assisted (Claude). Everything under
`src/` and `comms_link/src/` was written and debugged without AI assistance.
