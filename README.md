# Resilient Autonomous Mission Computer and Communications Simulator

A software simulation of mission computing and resilient communications, implemented in C++20, C11, and Rust. The mission computer combines IMU/GNSS sensor fusion, A* route planning, PID command generation, and watchdog-driven mission-state management. A separate Rust simulator evaluates sequenced message delivery and automatic link failover during a primary-link outage.

**Technologies:** C++20, C11, Rust, Python, CMake, Cargo, Linux, Docker build configuration.

**Algorithms and systems:** Kalman filtering, A* search, PID control, bounded message queues, heartbeat monitoring, fault injection, acknowledgement timeouts, and duplicate suppression.

## Technical report

**A Simulation-Based Mission Computer and Resilient Communications Platform: Architecture, Fault Injection, and Evaluation**

**[Read the full report in GitHub's PDF viewer](docs/comms-technical-paper.pdf)** · [Download PDF](docs/comms-technical-paper.pdf?raw=true) · [LaTeX source](docs/comms-technical-paper.tex)

<details>
<summary><strong>Preview the report: abstract, results, and scope</strong></summary>

### Abstract

The report examines a mission-computer simulation and a separate communications simulator implemented in C++20, C11, and Rust. It describes IMU/GNSS estimation, A* planning, PID command generation, watchdog supervision, sequenced message delivery, and score-based link failover. Evaluation combines source inspection with numerical analysis of committed outputs at a fixed repository revision.

### Results at a glance

| Evaluation | Recorded result |
|---|---|
| Fault-scenario final-state checks | 500 / 500 satisfied across four fault classes |
| Position estimation | 0.138 m planar RMSE across 1,500 trajectory samples |
| Communications failover | 22–41 ms simulated switching latency across 20 trials |

### Scope and findings

The report includes the architecture, estimator equations, algorithm choices, evaluation method, reproducibility commands, and a verification roadmap. It also documents the current actuation, integration, probability-model, and test-coverage limitations. Results describe archived software simulations; the executables were not independently rerun for the report.

[Continue to the complete PDF](docs/comms-technical-paper.pdf)

</details>

## Recorded results

The following values describe the committed artifacts at revision [`e04681f`](https://github.com/tyreefranklinjr/Mission-Computer--Communications-Platform/tree/e04681fdf181b30d5230861b9ff7cf7fef1f30c1). They are archived simulation results, not measurements from physical hardware. The numerical summaries below were recalculated from those CSV files; the executables were not rerun for this documentation update.

| Metric | Result | Evidence |
|---|---|---|
| Fault-scenario final-state checks | **500 / 500 passed** across four fault classes | [`fault_sweep_results.csv`](samples/fault_sweep_results.csv) |
| Planar position RMSE | **0.138 m** across 1,500 samples | [`ekf_trajectory.csv`](samples/ekf_trajectory.csv) |
| Position error below 0.3 m | **1,490 / 1,500 samples (99.33%)** | Full recorded trajectory, including initialization |
| Link-switching latency | **22–41 ms simulated**, mean **39 ms**; all 20 recorded trials below 250 ms | [`comms_link_failover_latencies.csv`](samples/comms_link_failover_latencies.csv) |
| Mission simulation step | **20 ms**, equivalent to a **50 Hz logical execution rate** | [`src/main.cpp`](src/main.cpp) |

The fault-sweep result includes 311 scenarios expected to enter `SAFE_STOP` and 189 expected to avoid it. It does not represent 500 safe-stop transitions. The existing checks evaluate the final mission state; they do not verify actuator inhibition or physical stopping.

The latency results measure the interval from primary-link outage onset to link selection. They do not measure hardware switching time or the time to the first successful application delivery. The current probability-model limitation is described under [Known limitations](#known-limitations).

![Simulation dashboard showing trajectory estimation, fault-sweep outcomes, and communications failover results](assets/dashboard.png)

The dashboard is generated from `samples/` by [`python/generate_report.py`](python/generate_report.py). The CSV files are the source of the tabulated results.

## Architecture

### Mission computer: C++ and C

One process executes four logical nodes sequentially on each simulated tick.

| Node | Responsibility | Main implementation |
|---|---|---|
| Sensor | Advance the scripted trajectory and publish noisy IMU/GNSS readings and a heartbeat | `imu_gnss_sim.hpp`, `main.cpp` |
| Navigation | Estimate planar position and velocity; plan routes on a 20 × 20 occupancy grid | `ekf.hpp`, `astar.hpp` |
| Control | Calculate a saturated PID command from the bearing to the next waypoint | `pid.hpp`, `main.cpp` |
| Supervisor | Check heartbeat freshness and GNSS availability; update the mission state | `watchdog.hpp`, `mission_state.hpp` |

The estimator maintains `[x, y, vx, vy]`. Although its class is named `Ekf`, the implemented motion and measurement models are linear. State prediction uses IMU acceleration; correction uses GNSS position. Matrix operations use fixed-size arrays.

A* uses four-connected, unit-cost moves and a Euclidean heuristic. Its frontier is a vector scanned for the lowest score. The mission maps grid coordinates to 2 m increments and requests replanning every 250 ticks, or whenever no path has been established.

The messaging layer is a standalone C library linked through `extern "C"`. It provides a 64-frame FIFO ring buffer with eight-byte payloads, bounded storage, and a dropped-message counter. Frame priority is metadata; the queue does not simulate CAN arbitration.

The supervisor advances through `INIT -> CALIBRATING -> AUTONOMOUS`. A communications outage can select `DEGRADED`, with recovery to `AUTONOMOUS` when the outage clears. An unhealthy core-system indication selects the terminal `SAFE_STOP` state.

### Communications simulator: Rust

`comms_link/` is an independent executable with:

- A `BinaryHeap` outgoing queue ordered by priority, then sequence number.
- Sequence tracking and receiver-side duplicate suppression.
- Acknowledgements, a 20 ms simulated timeout, and at most five total transmission attempts per message.
- A six-outcome rolling success score for each link.
- Automatic primary-to-secondary failover when the primary score falls below 0.5 after at least three recorded outcomes.
- A 30 s simulated primary-link outage within a 40 s scenario.

The Rust executable does not carry messages from the C++ mission computer. Its measurements describe a separate communications experiment.

## Build and run

### Requirements

- A C11 and C++20 compiler.
- CMake 3.16 or newer and a supported build tool.
- Rust and Cargo.
- Python 3 and the packages in `python/requirements.txt` for dashboard generation.
- Bash for the communications trial script.

Run the following commands from the repository root unless indicated otherwise.

### Mission computer

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure

mkdir -p evaluation
./build/mission_computer sim evaluation/trajectory.csv
./build/mission_computer fault-sweep 500 evaluation/faults.csv
```

`sim` executes 1,500 ticks, representing 30 s of simulated motion. `fault-sweep` creates a fresh 500-tick mission for each scenario. Outputs in `evaluation/` keep these runs separate from the committed baseline.

### Communications simulator

```bash
cd comms_link
cargo build --release --locked
./target/release/comms_link
bash run_failover_trials.sh
```

The default seed-7 run writes `../samples/comms_link_log.csv`. The trial script runs 20 seeds and writes `../samples/comms_link_failover_latencies.csv`. These commands replace the corresponding baseline files, so use a disposable checkout when preserving archived results matters.

### Dashboard

From the repository root:

```bash
python3 -m pip install -r python/requirements.txt
python3 python/generate_report.py
```

The report script reads `samples/*.csv` and replaces `assets/dashboard.png`. It does not automatically read the separate `evaluation/` outputs.

### Docker

```bash
docker build -t autonomy_platform .
docker run --rm autonomy_platform sim
docker run --rm autonomy_platform fault-sweep 500
```

The Dockerfile is included as build configuration. The repository's development notes report that it was hand-reviewed without a live Docker daemon; this documentation update does not establish a successful container build.

## Fault injection and verification

`fault_injector.hpp` derives fault type, injection time, duration, and expected outcome from the scenario number. The harness covers GNSS dropout, CAN publication failure, network outage, and skipped control-task execution.

| Fault case | Recorded scenarios | Expected behavior | Recorded reaction interval |
|---|---:|---|---|
| CAN publication failure | 125 | Enter `SAFE_STOP` | 5 ticks / 100 ms |
| Skipped control task | 125 | Enter `SAFE_STOP` | 5 ticks / 100 ms |
| Long GNSS dropout | 61 | Enter `SAFE_STOP` | 40 ticks / 800 ms |
| Short GNSS dropout | 64 | Avoid `SAFE_STOP` | Not applicable |
| Network outage | 125 | Avoid `SAFE_STOP` | Not applicable |

Reaction intervals start at the injection tick. The watchdog becomes unhealthy when heartbeat age exceeds five ticks. The five-tick recorded delay begins with the first omitted heartbeat, one tick after the last successful heartbeat. GNSS safing occurs on the forty-first consecutive missing sample, 40 tick intervals after the first missing sample.

The CTest targets exercise estimator convergence and small-grid planning. The fault sweep checks final states. For tolerated faults, either `AUTONOMOUS` or `DEGRADED` is accepted, so passing does not independently prove a complete degrade-and-recover sequence.

## Simulation notes

- Time advances through logical loop iterations. The 50 Hz mission rate and millisecond communications timings do not establish wall-clock deadline compliance.
- All logical mission nodes execute in one process. Skipping the control function models missing work and heartbeats, rather than a real task blocking its scheduler.
- Sensor readings are generated from scripted motion plus noise. Physical sensors, radios, ESP32-C3 hardware, and FreeRTOS are outside the current implementation.
- The in-memory message queue models CAN-style frames without physical timing, arbitration, or transceiver behavior.

## Known limitations

- **Actuation and control:** `SAFE_STOP` is a latched mission-state decision, but it does not inhibit actuator-command generation. PID output does not drive the scripted trajectory, and its input is a waypoint bearing without subtraction of vehicle heading. Closed-loop tracking and stopping behavior have not been demonstrated.
- **Communications probabilities:** The Rust generator retains 31 random bits but divides by a 32-bit maximum, producing values approximately in `[0, 0.5)`. Configured loss and late-arrival thresholds therefore differ from their intended probabilities. The archived results describe this implementation; correcting the generator requires a new baseline.
- **Recovery and replay:** The inactive primary has no explicit health-probing mechanism for failback. Randomized `HashMap` iteration can also affect the order in which resolved outcomes enter the score history, so a fixed seed alone does not guarantee identical event traces.
- **Load and coverage:** The Rust loop drains its outgoing queue without a bandwidth constraint. Priority ordering does not establish performance under congestion. Existing tests also leave gaps in actuator inhibition, transition deadlines, queue saturation, grid-dimension validation, and complete path-adjacency checks.

The next verification steps are to connect safing to actuation, correct and rebaseline the communications model, assert transition sequences and deadlines, and integrate the two executables through an explicit message interface.

## Repository layout

| Path | Contents |
|---|---|
| `src/` | Mission computer, estimator, planner, controller, watchdog, state machine, fault injection, and C message queue |
| `tests/` | Estimator and A* test executables |
| `comms_link/` | Rust communications simulator and seeded trial script |
| `python/` | Dashboard generation and Python requirements |
| `samples/` | Committed trajectory, fault-sweep, communications event, and latency CSV files |
| `assets/` | Generated simulation dashboard |

## Development notes

Only the files under `tests/` and `python/` were AI-assisted (Claude). Everything under `src/` and `comms_link/src/` was written and debugged without AI assistance. This README revision was prepared with AI assistance.

## License

[MIT](LICENSE).
