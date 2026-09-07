// mission_state.hpp
//
// The mission-level state machine. This is the piece that turns "a node
// missed its watchdog" or "the CAN bus is faulted" into a system-wide
// decision: keep flying autonomously, degrade gracefully, or stop in a
// known-safe state.

#ifndef AUTONOMY_PLATFORM_MISSION_STATE_HPP
#define AUTONOMY_PLATFORM_MISSION_STATE_HPP

using namespace std;

enum MissionState {
    MISSION_STATE_INIT = 0,
    MISSION_STATE_CALIBRATING = 1,
    MISSION_STATE_AUTONOMOUS = 2,
    MISSION_STATE_DEGRADED = 3,
    MISSION_STATE_SAFE_STOP = 4
};

const char *MissionStateName(MissionState state) {
    if (state == MISSION_STATE_INIT) {
        return "INIT";
    }
    if (state == MISSION_STATE_CALIBRATING) {
        return "CALIBRATING";
    }
    if (state == MISSION_STATE_AUTONOMOUS) {
        return "AUTONOMOUS";
    }
    if (state == MISSION_STATE_DEGRADED) {
        return "DEGRADED";
    }
    return "SAFE_STOP";
}

class MissionStateMachine {
public:
    MissionStateMachine(int calibration_ticks) {
        state_ = MISSION_STATE_INIT;
        calibration_ticks_ = calibration_ticks;
        ticks_in_state_ = 0;
    }

    MissionState GetState() const {
        return state_;
    }

    // Advances the state machine by one tick given the current health
    // picture. system_healthy covers watchdog + CAN bus faults,
    // comms_healthy covers the degraded-communications link (bullet 2).
    void Update(bool system_healthy, bool comms_healthy) {
        ticks_in_state_ = ticks_in_state_ + 1;

        if (!system_healthy) {
            // Any core system fault (a node stopped feeding its watchdog,
            // or the CAN bus itself faulted) is treated as unsafe to
            // continue autonomously, regardless of prior state.
            TransitionTo(MISSION_STATE_SAFE_STOP);
            return;
        }

        if (state_ == MISSION_STATE_INIT) {
            TransitionTo(MISSION_STATE_CALIBRATING);
            return;
        }

        if (state_ == MISSION_STATE_CALIBRATING) {
            if (ticks_in_state_ >= calibration_ticks_) {
                TransitionTo(MISSION_STATE_AUTONOMOUS);
            }
            return;
        }

        if (state_ == MISSION_STATE_AUTONOMOUS) {
            if (!comms_healthy) {
                TransitionTo(MISSION_STATE_DEGRADED);
            }
            return;
        }

        if (state_ == MISSION_STATE_DEGRADED) {
            if (comms_healthy) {
                TransitionTo(MISSION_STATE_AUTONOMOUS);
            }
            return;
        }

        // MISSION_STATE_SAFE_STOP is terminal for this simulation run: once
        // the system has safed itself it stays safed rather than silently
        // resuming, matching how a real safety-critical mission computer
        // would require an explicit reset before flying again.
    }

private:
    void TransitionTo(MissionState next_state) {
        if (next_state != state_) {
            state_ = next_state;
            ticks_in_state_ = 0;
        }
    }

    MissionState state_;
    int calibration_ticks_;
    int ticks_in_state_;
};

#endif // AUTONOMY_PLATFORM_MISSION_STATE_HPP
