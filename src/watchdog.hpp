// watchdog.hpp
//
// A simple software watchdog. Each simulated node is expected to "feed"
// (heartbeat) the watchdog once per scheduler tick. If a node misses its
// heartbeat for longer than a timeout, the watchdog reports it as unhealthy
// so the mission state machine can react (typically by dropping to a safe
// state). This models the watchdog behavior a real embedded node would
// implement, but here it just tracks tick counts in memory, there is no
// hardware timer or reset line.

#ifndef AUTONOMY_PLATFORM_WATCHDOG_HPP
#define AUTONOMY_PLATFORM_WATCHDOG_HPP

using namespace std;

#define WATCHDOG_MAX_NODES 4

class Watchdog {
public:
    Watchdog(int timeout_ticks) {
        timeout_ticks_ = timeout_ticks;
        for (int i = 0; i < WATCHDOG_MAX_NODES; i = i + 1) {
            last_fed_tick_[i] = 0;
        }
    }

    // Called by a node each time it completes its work for the current tick.
    void Feed(int node_id, int current_tick) {
        if (node_id >= 0 && node_id < WATCHDOG_MAX_NODES) {
            last_fed_tick_[node_id] = current_tick;
        }
    }

    // Returns true if the node has fed the watchdog within timeout_ticks_.
    bool IsHealthy(int node_id, int current_tick) const {
        if (node_id < 0 || node_id >= WATCHDOG_MAX_NODES) {
            return false;
        }
        return (current_tick - last_fed_tick_[node_id]) <= timeout_ticks_;
    }

    // Returns true if every node in [0, node_count) is currently healthy.
    bool AllHealthy(int node_count, int current_tick) const {
        for (int i = 0; i < node_count; i = i + 1) {
            if (!IsHealthy(i, current_tick)) {
                return false;
            }
        }
        return true;
    }

private:
    int last_fed_tick_[WATCHDOG_MAX_NODES];
    int timeout_ticks_;
};

#endif // AUTONOMY_PLATFORM_WATCHDOG_HPP
