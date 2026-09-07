// main.cpp
//
// Entry point for the autonomy platform simulation. Everything in this
// program runs as ONE ordinary process on a regular Linux machine. There is
// no ESP32-C3, no physical CAN bus, and FreeRTOS itself is not linked in;
// see README.md's "Simulation notes" section for the full explanation of
// what is simulated and why.
//
// Two modes:
//   sim [trajectory_log.csv]
//       Runs one continuous simulated mission and prints a live report.
//   fault-sweep [num_scenarios] [results.csv]
//       Runs the deterministic SIL/HIL fault-injection harness across
//       num_scenarios (default 500) scenarios and reports the safe-state
//       transition success rate.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

using namespace std;

#include "imu_gnss_sim.hpp"
#include "ekf.hpp"
#include "pid.hpp"
#include "astar.hpp"
#include "watchdog.hpp"
#include "mission_state.hpp"
#include "fault_injector.hpp"
#include "metrics.hpp"

extern "C" {
#include "can_bus.h"
}

// ---- constants -------------------------------------------------------

const float kTickPeriodSeconds = 0.02f;   // 50 Hz, the fastest rate bullet 1 claims
const int kNodeSensor = 0;
const int kNodeNav = 1;
const int kNodeControl = 2;
const int kWatchdogTimeoutTicks = 5;       // 100 ms at 50 Hz
const int kCalibrationTicks = 25;          // 0.5 s settle time before AUTONOMOUS
const int kSensorSafeDropoutTicks = 40;    // > 0.8 s with no GNSS fix is unsafe
const int kPlanReplanIntervalTicks = 250;  // replan roughly every 5 s

const unsigned int kCanIdImu = 0x100;
const unsigned int kCanIdGnss = 0x101;
const unsigned int kCanIdEstimate = 0x200;
const unsigned int kCanIdActuator = 0x300;
const unsigned int kCanIdHeartbeatBase = 0x0F0; // + node id

// ---- small CAN payload helpers ----------------------------------------

// Packs two floats into an 8-byte CAN payload (the whole payload).
void PackTwoFloats(CanMessage *message, unsigned int id, uint8_t priority, float a, float b) {
    message->id = id;
    message->priority = priority;
    message->dlc = 8;
    memcpy(&message->payload[0], &a, sizeof(float));
    memcpy(&message->payload[4], &b, sizeof(float));
}

void UnpackTwoFloats(const CanMessage *message, float *out_a, float *out_b) {
    memcpy(out_a, &message->payload[0], sizeof(float));
    memcpy(out_b, &message->payload[4], sizeof(float));
}

void PublishHeartbeat(CanBus *bus, int node_id) {
    CanMessage message;
    message.id = kCanIdHeartbeatBase + (unsigned int)node_id;
    message.priority = 0;
    message.dlc = 1;
    message.payload[0] = (uint8_t)node_id;
    CanBusPublish(bus, &message);
}

// ---- the simulated mission computer ------------------------------------

// Holds every piece of state for one simulated mission run: the 4 logical
// nodes (sensor, navigation, control, supervisor), the simulated CAN bus
// they talk over, and the fault flags the fault-sweep harness can toggle.
// Each call to RunTick steps every node once, the way a fixed-rate
// scheduler would dispatch each task once per period.
class MissionComputerSim {
public:
    MissionComputerSim(unsigned int seed)
        : sensor_sim_(seed),
          watchdog_(kWatchdogTimeoutTicks),
          mission_state_(kCalibrationTicks),
          grid_(20, 20) {
        CanBusInit(&can_bus_);

        grid_.SetObstacle(5, 5);
        grid_.SetObstacle(5, 6);
        grid_.SetObstacle(5, 7);
        grid_.SetObstacle(10, 3);
        grid_.SetObstacle(10, 4);
        grid_.SetObstacle(10, 5);

        goal_.x = 18;
        goal_.y = 18;

        have_path_ = false;
        path_index_ = 0;

        last_imu_ax_ = 0.0f;
        last_imu_ay_ = 0.0f;
        last_gnss_x_ = 0.0f;
        last_gnss_y_ = 0.0f;
        gnss_updated_this_tick_ = false;

        consecutive_gnss_dropout_ticks_ = 0;
        control_node_hung_ = false;
        network_outage_active_ = false;

        heading_pid_ = new PidController(1.2f, 0.02f, 0.15f, 2.0f);

        last_actuator_command_ = 0.0f;
    }

    ~MissionComputerSim() {
        delete heading_pid_;
    }

    // Fault injection hooks, used by both `sim` (not exercised there) and
    // `fault-sweep`.
    void SetSensorFault(bool active) {
        sensor_sim_.SetGnssDropout(active);
    }

    void SetCanBusFault(bool active) {
        CanBusSetFaulted(&can_bus_, active ? 1 : 0);
    }

    void SetControlNodeHung(bool hung) {
        control_node_hung_ = hung;
    }

    void SetNetworkOutage(bool active) {
        network_outage_active_ = active;
    }

    MissionState GetMissionState() const {
        return mission_state_.GetState();
    }

    TrueState GetTrueState() const {
        return sensor_sim_.GetTrueState();
    }

    EkfState GetEstimatedState() const {
        return ekf_.GetState();
    }

    // Runs one 20 ms tick: sensor node, navigation node, control node,
    // supervisor node, in that fixed order, mirroring a cooperative
    // fixed-rate scheduler dispatching 4 periodic tasks once per period.
    void RunTick(int tick) {
        RunSensorNode(tick);
        RunNavNode(tick);
        RunControlNode(tick);
        RunSupervisorNode(tick);
    }

private:
    void RunSensorNode(int tick) {
        sensor_sim_.Step(kTickPeriodSeconds);

        ImuSample imu = sensor_sim_.ReadImu();
        GnssSample gnss = sensor_sim_.ReadGnss();

        if (imu.valid) {
            CanMessage message;
            PackTwoFloats(&message, kCanIdImu, 1, imu.ax, imu.ay);
            CanBusPublish(&can_bus_, &message);
        }

        if (gnss.valid) {
            CanMessage message;
            PackTwoFloats(&message, kCanIdGnss, 2, gnss.x, gnss.y);
            CanBusPublish(&can_bus_, &message);
            consecutive_gnss_dropout_ticks_ = 0;
        } else {
            consecutive_gnss_dropout_ticks_ = consecutive_gnss_dropout_ticks_ + 1;
        }

        PublishHeartbeat(&can_bus_, kNodeSensor);
    }

    void RunNavNode(int tick) {
        gnss_updated_this_tick_ = false;

        // Drain exactly the messages that are queued right now (not
        // anything requeued below), so a message that isn't for this node
        // is put back for the next node without looping forever.
        int pending = CanBusPending(&can_bus_);
        for (int i = 0; i < pending; i = i + 1) {
            CanMessage message;
            if (!CanBusReceive(&can_bus_, &message)) {
                break;
            }
            if (message.id == kCanIdImu) {
                UnpackTwoFloats(&message, &last_imu_ax_, &last_imu_ay_);
            } else if (message.id == kCanIdGnss) {
                UnpackTwoFloats(&message, &last_gnss_x_, &last_gnss_y_);
                gnss_updated_this_tick_ = true;
            } else {
                // Not this node's message; every node on a real bus sees
                // every frame and filters by id, so put it back for the
                // next node to see this tick.
                RequeueForOthers(message);
            }
        }

        ekf_.Predict(last_imu_ax_, last_imu_ay_, kTickPeriodSeconds);
        if (gnss_updated_this_tick_) {
            ekf_.UpdateWithGnss(last_gnss_x_, last_gnss_y_);
        }

        if (!have_path_ || tick % kPlanReplanIntervalTicks == 0) {
            EkfState estimate = ekf_.GetState();
            GridCell start;
            start.x = (int)(estimate.x / 2.0f);
            start.y = (int)(estimate.y / 2.0f);
            if (start.x < 0) start.x = 0;
            if (start.y < 0) start.y = 0;
            if (start.x >= grid_.GetWidth()) start.x = grid_.GetWidth() - 1;
            if (start.y >= grid_.GetHeight()) start.y = grid_.GetHeight() - 1;

            vector<GridCell> path;
            bool found = AstarFindPath(grid_, start, goal_, path);
            if (found) {
                current_path_ = path;
                path_index_ = 0;
                have_path_ = true;
            }
        }

        EkfState estimate = ekf_.GetState();
        CanMessage estimate_message;
        PackTwoFloats(&estimate_message, kCanIdEstimate, 1, estimate.x, estimate.y);
        CanBusPublish(&can_bus_, &estimate_message);

        PublishHeartbeat(&can_bus_, kNodeNav);
    }

    void RunControlNode(int tick) {
        if (control_node_hung_) {
            // Simulates a task overrun / hang: this node does no work and,
            // critically, does not feed the watchdog this tick.
            return;
        }

        float estimate_x = 0.0f;
        float estimate_y = 0.0f;
        bool got_estimate = false;

        int pending = CanBusPending(&can_bus_);
        for (int i = 0; i < pending; i = i + 1) {
            CanMessage message;
            if (!CanBusReceive(&can_bus_, &message)) {
                break;
            }
            if (message.id == kCanIdEstimate) {
                UnpackTwoFloats(&message, &estimate_x, &estimate_y);
                got_estimate = true;
            } else {
                RequeueForOthers(message);
            }
        }

        if (got_estimate && have_path_ && path_index_ < (int)current_path_.size()) {
            GridCell waypoint = current_path_[path_index_];
            float target_x = (float)waypoint.x * 2.0f;
            float target_y = (float)waypoint.y * 2.0f;

            float dx = target_x - estimate_x;
            float dy = target_y - estimate_y;
            float distance = sqrtf(dx * dx + dy * dy);

            if (distance < 1.0f && path_index_ < (int)current_path_.size() - 1) {
                path_index_ = path_index_ + 1;
            }

            float heading_error = atan2f(dy, dx);
            last_actuator_command_ = heading_pid_->Update(heading_error, kTickPeriodSeconds);

            CanMessage actuator_message;
            actuator_message.id = kCanIdActuator;
            actuator_message.priority = 1;
            actuator_message.dlc = 4;
            memcpy(&actuator_message.payload[0], &last_actuator_command_, sizeof(float));
            CanBusPublish(&can_bus_, &actuator_message);
        }

        PublishHeartbeat(&can_bus_, kNodeControl);
    }

    void RunSupervisorNode(int tick) {
        CanMessage message;
        while (CanBusReceive(&can_bus_, &message)) {
            if (message.id == kCanIdHeartbeatBase + (unsigned int)kNodeSensor) {
                watchdog_.Feed(kNodeSensor, tick);
            } else if (message.id == kCanIdHeartbeatBase + (unsigned int)kNodeNav) {
                watchdog_.Feed(kNodeNav, tick);
            } else if (message.id == kCanIdHeartbeatBase + (unsigned int)kNodeControl) {
                watchdog_.Feed(kNodeControl, tick);
            }
            // Any other message left over this tick is discarded; the
            // supervisor only cares about heartbeats.
        }

        bool nodes_healthy = watchdog_.IsHealthy(kNodeSensor, tick)
                && watchdog_.IsHealthy(kNodeNav, tick)
                && watchdog_.IsHealthy(kNodeControl, tick);
        bool sensor_safe = consecutive_gnss_dropout_ticks_ <= kSensorSafeDropoutTicks;
        bool system_healthy = nodes_healthy && sensor_safe;
        bool comms_healthy = !network_outage_active_;

        mission_state_.Update(system_healthy, comms_healthy);
    }

    // A message this tick's node loop did not recognize gets published
    // straight back so the next node in the same tick can still see it.
    // This keeps the demo simple (one shared bus, drained fully by each
    // node each tick) while still routing messages by id.
    void RequeueForOthers(const CanMessage &message) {
        CanBusPublish(&can_bus_, &message);
    }

public:
    SensorSimulator sensor_sim_;
    Ekf ekf_;
    Watchdog watchdog_;
    MissionStateMachine mission_state_;
    OccupancyGrid grid_;
    CanBus can_bus_;

    GridCell goal_;
    vector<GridCell> current_path_;
    bool have_path_;
    int path_index_;

    float last_imu_ax_;
    float last_imu_ay_;
    float last_gnss_x_;
    float last_gnss_y_;
    bool gnss_updated_this_tick_;

    int consecutive_gnss_dropout_ticks_;
    bool control_node_hung_;
    bool network_outage_active_;

    PidController *heading_pid_;
    float last_actuator_command_;
};

// ---- sim mode -----------------------------------------------------------

int RunSimMode(int total_ticks, const char *trajectory_log_path) {
    printf("Running continuous mission simulation for %d ticks (%.1f s simulated at 50 Hz).\n",
           total_ticks, (float)total_ticks * kTickPeriodSeconds);
    printf("This is a software simulation running on Linux. No physical ESP32-C3, CAN bus,\n");
    printf("or FreeRTOS target is involved. See README.md for details.\n\n");

    MissionComputerSim sim(42u);

    FILE *log_file = nullptr;
    if (trajectory_log_path != nullptr) {
        log_file = fopen(trajectory_log_path, "w");
        if (log_file != nullptr) {
            WriteTrajectoryLogHeader(log_file);
        }
    }

    for (int tick = 0; tick < total_ticks; tick = tick + 1) {
        sim.RunTick(tick);

        if (log_file != nullptr) {
            TrueState true_state = sim.GetTrueState();
            EkfState estimate = sim.GetEstimatedState();
            fprintf(log_file, "%d,%.4f,%.4f,%.4f,%.4f,%s\n",
                    tick, true_state.x, true_state.y, estimate.x, estimate.y,
                    MissionStateName(sim.GetMissionState()));
        }

        if (tick % 50 == 0) {
            TrueState true_state = sim.GetTrueState();
            EkfState estimate = sim.GetEstimatedState();
            float error_x = estimate.x - true_state.x;
            float error_y = estimate.y - true_state.y;
            float error = sqrtf(error_x * error_x + error_y * error_y);
            printf("t=%6.2fs  state=%-11s  true=(%.2f, %.2f)  est=(%.2f, %.2f)  pos_error=%.3f\n",
                   (float)tick * kTickPeriodSeconds,
                   MissionStateName(sim.GetMissionState()),
                   true_state.x, true_state.y, estimate.x, estimate.y, error);
        }
    }

    if (log_file != nullptr) {
        fclose(log_file);
        printf("\nTrajectory log written to %s\n", trajectory_log_path);
    }

    printf("\nFinal mission state: %s\n", MissionStateName(sim.GetMissionState()));
    return 0;
}

// ---- fault-sweep mode -----------------------------------------------------

int RunFaultSweepMode(int num_scenarios, const char *results_csv_path) {
    const int kSimLengthTicks = 500; // 10 s simulated per scenario
    const int kSafeStopVerifyBuffer = 40;

    printf("Running deterministic SIL/HIL fault-injection sweep: %d scenarios.\n", num_scenarios);
    printf("Each scenario is a fresh %d-tick (%.1f s simulated) mission with one injected fault.\n\n",
           kSimLengthTicks, (float)kSimLengthTicks * kTickPeriodSeconds);

    vector<ScenarioResult> results;
    int success_count = 0;

    for (int scenario_id = 0; scenario_id < num_scenarios; scenario_id = scenario_id + 1) {
        FaultScenario scenario = GenerateFaultScenario(scenario_id, kSimLengthTicks);
        MissionComputerSim sim((unsigned int)(scenario_id + 1000));

        MissionState state_when_reacted = MISSION_STATE_INIT;
        int reacted_tick = -1;
        int fault_end_tick = scenario.inject_tick + scenario.duration_ticks;

        for (int tick = 0; tick < kSimLengthTicks; tick = tick + 1) {
            bool fault_active = tick >= scenario.inject_tick && tick < fault_end_tick;

            if (scenario.type == FAULT_SENSOR_DROPOUT) {
                sim.SetSensorFault(fault_active);
            } else if (scenario.type == FAULT_CAN_BUS) {
                sim.SetCanBusFault(fault_active);
            } else if (scenario.type == FAULT_NETWORK_OUTAGE) {
                sim.SetNetworkOutage(fault_active);
            } else {
                sim.SetControlNodeHung(fault_active);
            }

            sim.RunTick(tick);

            if (reacted_tick < 0 && sim.GetMissionState() == MISSION_STATE_SAFE_STOP) {
                reacted_tick = tick;
                state_when_reacted = sim.GetMissionState();
            }
        }

        MissionState final_state = sim.GetMissionState();
        bool reached_expected_state;
        if (scenario.expect_safe_stop) {
            reached_expected_state = (final_state == MISSION_STATE_SAFE_STOP);
        } else {
            bool never_incorrectly_safed = (final_state != MISSION_STATE_SAFE_STOP);
            bool recovered = (final_state == MISSION_STATE_AUTONOMOUS || final_state == MISSION_STATE_DEGRADED);
            reached_expected_state = never_incorrectly_safed && recovered;
        }

        int ticks_to_react = 0;
        if (scenario.expect_safe_stop && reacted_tick >= 0) {
            ticks_to_react = reacted_tick - scenario.inject_tick;
        }

        ScenarioResult result;
        result.scenario_id = scenario.scenario_id;
        result.fault_type = scenario.type;
        result.inject_tick = scenario.inject_tick;
        result.duration_ticks = scenario.duration_ticks;
        result.expected_safe_stop = scenario.expect_safe_stop;
        result.reached_expected_state = reached_expected_state;
        result.ticks_to_react = ticks_to_react;
        result.final_state = final_state;
        results.push_back(result);

        if (reached_expected_state) {
            success_count = success_count + 1;
        }

        (void)kSafeStopVerifyBuffer;
        (void)state_when_reacted;
    }

    float success_rate = 100.0f * (float)success_count / (float)num_scenarios;

    printf("Scenarios run:            %d\n", num_scenarios);
    printf("Safe-state transitions correct: %d / %d (%.2f%%)\n", success_count, num_scenarios, success_rate);

    for (int type_index = 0; type_index < FAULT_TYPE_COUNT; type_index = type_index + 1) {
        int type_total = 0;
        int type_success = 0;
        for (int i = 0; i < (int)results.size(); i = i + 1) {
            if ((int)results[i].fault_type == type_index) {
                type_total = type_total + 1;
                if (results[i].reached_expected_state) {
                    type_success = type_success + 1;
                }
            }
        }
        if (type_total > 0) {
            printf("  %-16s %d / %d\n", FaultTypeName((FaultType)type_index), type_success, type_total);
        }
    }

    if (results_csv_path != nullptr) {
        bool written = WriteScenarioResultsCsv(results, results_csv_path);
        if (written) {
            printf("\nPer-scenario results written to %s\n", results_csv_path);
        } else {
            printf("\nWARNING: failed to write results CSV to %s\n", results_csv_path);
        }
    }

    return (success_count == num_scenarios) ? 0 : 1;
}

// ---- entry point -----------------------------------------------------

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage:\n");
        printf("  %s sim [trajectory_log.csv]\n", argv[0]);
        printf("  %s fault-sweep [num_scenarios] [results.csv]\n", argv[0]);
        return 1;
    }

    if (strcmp(argv[1], "sim") == 0) {
        const char *log_path = (argc >= 3) ? argv[2] : nullptr;
        return RunSimMode(1500, log_path);
    }

    if (strcmp(argv[1], "fault-sweep") == 0) {
        int num_scenarios = 500;
        if (argc >= 3) {
            num_scenarios = atoi(argv[2]);
            if (num_scenarios < 1) {
                num_scenarios = 500;
            }
        }
        const char *csv_path = (argc >= 4) ? argv[3] : nullptr;
        return RunFaultSweepMode(num_scenarios, csv_path);
    }

    printf("Unknown mode '%s'. Use 'sim' or 'fault-sweep'.\n", argv[1]);
    return 1;
}
