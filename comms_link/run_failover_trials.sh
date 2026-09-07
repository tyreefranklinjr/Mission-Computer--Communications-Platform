#!/usr/bin/env bash
# run_failover_trials.sh
#
# Runs the comms_link simulation with several different RNG seeds so the
# analytics dashboard has more than one failover-latency sample to plot.
# Seed 7 is the "main" run (its full event log is kept in samples/); the
# other seeds here only contribute a failover-latency measurement.
#
# Writes samples/comms_link_failover_latencies.csv.

set -euo pipefail

cd "$(dirname "$0")"

OUT_FILE="../samples/comms_link_failover_latencies.csv"
echo "seed,failover_latency_ms" > "$OUT_FILE"

for seed in 7 11 13 17 19 23 29 31 37 41 43 47 53 59 61 67 71 73 79 83; do
    latency=$(./target/release/comms_link "$seed" | grep "Failover latency" | grep -oE '[0-9]+ ms' | head -1 | grep -oE '[0-9]+')
    echo "${seed},${latency}" >> "$OUT_FILE"
done

echo "Wrote $OUT_FILE"
cat "$OUT_FILE"
