// metrics.hpp
//
// Small plain-C-style helpers for writing simulation results to CSV files
// using FILE* (fopen/fwrite/fclose), matching the companion project's
// preference for C-style file I/O over C++ streams.

#ifndef AUTONOMY_PLATFORM_METRICS_HPP
#define AUTONOMY_PLATFORM_METRICS_HPP

#include <cstdio>
#include <vector>

#include "fault_injector.hpp"
#include "mission_state.hpp"

using namespace std;

// Outcome of running one fault-sweep scenario against the simulation.
struct ScenarioResult {
    int scenario_id;
    FaultType fault_type;
    int inject_tick;
    int duration_ticks;
    bool expected_safe_stop;
    bool reached_expected_state;
    int ticks_to_react;
    MissionState final_state;
};

// Writes the full set of fault-sweep results to a CSV file.
bool WriteScenarioResultsCsv(const vector<ScenarioResult> &results, const char *file_path) {
    FILE *file = fopen(file_path, "w");
    if (file == nullptr) {
        return false;
    }

    fprintf(file, "scenario_id,fault_type,inject_tick,duration_ticks,expected_safe_stop,reached_expected_state,ticks_to_react,final_state\n");

    for (int i = 0; i < (int)results.size(); i = i + 1) {
        const ScenarioResult &result = results[i];
        fprintf(file, "%d,%s,%d,%d,%d,%d,%d,%s\n",
                result.scenario_id,
                FaultTypeName(result.fault_type),
                result.inject_tick,
                result.duration_ticks,
                result.expected_safe_stop ? 1 : 0,
                result.reached_expected_state ? 1 : 0,
                result.ticks_to_react,
                MissionStateName(result.final_state));
    }

    fclose(file);
    return true;
}

// Writes one row of a simulated-vs-true trajectory log (used for the
// EKF plot in the analytics dashboard).
bool WriteTrajectoryLogHeader(FILE *file) {
    if (file == nullptr) {
        return false;
    }
    fprintf(file, "tick,true_x,true_y,est_x,est_y,mission_state\n");
    return true;
}

#endif // AUTONOMY_PLATFORM_METRICS_HPP
