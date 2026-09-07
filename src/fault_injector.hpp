// fault_injector.hpp
//
// Deterministic fault generation for the SIL/HIL fault-sweep harness
// (Software-In-the-Loop / Hardware-In-the-Loop style testing, run entirely
// in software here since there is no hardware in the loop). Each scenario
// is derived only from its scenario number, so running the sweep twice
// produces exactly the same scenarios and results.

#ifndef AUTONOMY_PLATFORM_FAULT_INJECTOR_HPP
#define AUTONOMY_PLATFORM_FAULT_INJECTOR_HPP

using namespace std;

enum FaultType {
    FAULT_SENSOR_DROPOUT = 0,
    FAULT_CAN_BUS = 1,
    FAULT_NETWORK_OUTAGE = 2,
    FAULT_SYSTEM_OVERRUN = 3,
    FAULT_TYPE_COUNT = 4
};

const char *FaultTypeName(FaultType type) {
    if (type == FAULT_SENSOR_DROPOUT) {
        return "sensor_dropout";
    }
    if (type == FAULT_CAN_BUS) {
        return "can_bus_fault";
    }
    if (type == FAULT_NETWORK_OUTAGE) {
        return "network_outage";
    }
    return "system_overrun";
}

// A single fault-sweep scenario: what kind of fault, when it starts, how
// long it lasts, and whether it is expected to force a safe-state stop
// (a "hard" fault) or be tolerated with graceful degradation ("soft" fault).
struct FaultScenario {
    int scenario_id;
    FaultType type;
    int inject_tick;
    int duration_ticks;
    bool expect_safe_stop;
};

// Generates scenario N deterministically from N alone using a small linear
// congruential generator seeded by the scenario id. No external randomness,
// so results are reproducible run to run.
FaultScenario GenerateFaultScenario(int scenario_id, int sim_length_ticks) {
    unsigned int state = (unsigned int)(scenario_id * 2654435761u + 1u);

    state = state * 1103515245u + 12345u;
    unsigned int type_pick = (state / 65536u) % (unsigned int)FAULT_TYPE_COUNT;

    state = state * 1103515245u + 12345u;
    unsigned int tick_pick = (state / 65536u) % 32768u;

    state = state * 1103515245u + 12345u;
    unsigned int duration_pick = (state / 65536u) % 32768u;

    state = state * 1103515245u + 12345u;
    unsigned int severity_pick = (state / 65536u) % 32768u;

    FaultScenario scenario;
    scenario.scenario_id = scenario_id;
    scenario.type = (FaultType)type_pick;

    int usable_window = sim_length_ticks - 200;
    if (usable_window < 50) {
        usable_window = 50;
    }
    scenario.inject_tick = 50 + (int)(tick_pick % (unsigned int)usable_window);

    if (scenario.type == FAULT_CAN_BUS || scenario.type == FAULT_SYSTEM_OVERRUN) {
        // Hard faults: the bus or a node itself is broken. These are always
        // expected to force a safe-state stop, regardless of duration.
        scenario.duration_ticks = 20 + (int)(duration_pick % 60u);
        scenario.expect_safe_stop = true;
    } else if (scenario.type == FAULT_SENSOR_DROPOUT) {
        // Soft fault, but a long enough sensor dropout is itself unsafe.
        // Roughly half the generated scenarios are "long" (past the
        // 40-tick safety threshold used in main.cpp) and half are short.
        bool long_dropout = (severity_pick % 2u) == 0u;
        if (long_dropout) {
            scenario.duration_ticks = 45 + (int)(duration_pick % 40u);
            scenario.expect_safe_stop = true;
        } else {
            scenario.duration_ticks = 5 + (int)(duration_pick % 20u);
            scenario.expect_safe_stop = false;
        }
    } else {
        // FAULT_NETWORK_OUTAGE: a communications outage. Bullet 2's story
        // is that the system stays autonomous through outages up to 30 s,
        // so within this sim window a network outage is always a soft
        // fault: expect graceful degrade-and-recover, never a safe stop.
        scenario.duration_ticks = 20 + (int)(duration_pick % 60u);
        scenario.expect_safe_stop = false;
    }

    return scenario;
}

#endif // AUTONOMY_PLATFORM_FAULT_INJECTOR_HPP
