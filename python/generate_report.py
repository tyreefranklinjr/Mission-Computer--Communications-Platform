#!/usr/bin/env python3
# generate_report.py
#
# AI-assisted (Claude): this analytics script was developed with AI assistance.
#
# Reads the real sample data produced by actual runs of the C++/C mission
# computer (fault-sweep and sim modes) and the Rust comms_link simulation,
# and produces assets/dashboard.png: a 3-panel figure with a failover
# latency histogram, a safe-state transition success-rate bar, and an EKF
# estimated-vs-true trajectory plot.
#
# This script does not run any simulation itself. Run the simulations
# first (see README.md "How to build and run"), then run this script.

import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Palette shared with the companion SDR project, for visual consistency
# across this person's two projects.
INK = "#1a1a2e"
INK_SECONDARY = "#4a4a68"
INK_MUTED = "#8888a0"
GRID = "#e4e4ec"
SURFACE = "#ffffff"
BLUE = "#2a78d6"
ORANGE = "#eb6834"

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
SAMPLES_DIR = os.path.join(REPO_ROOT, "samples")
ASSETS_DIR = os.path.join(REPO_ROOT, "assets")

FAULT_SWEEP_CSV = os.path.join(SAMPLES_DIR, "fault_sweep_results.csv")
FAILOVER_LATENCIES_CSV = os.path.join(SAMPLES_DIR, "comms_link_failover_latencies.csv")
TRAJECTORY_CSV = os.path.join(SAMPLES_DIR, "ekf_trajectory.csv")
OUTPUT_PATH = os.path.join(ASSETS_DIR, "dashboard.png")


def read_fault_sweep_success_rate(path):
    total = 0
    correct = 0
    with open(path, "r") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            total = total + 1
            if row["reached_expected_state"] == "1":
                correct = correct + 1
    return correct, total


def read_failover_latencies(path):
    latencies = []
    with open(path, "r") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            latencies.append(int(row["failover_latency_ms"]))
    return latencies


def read_trajectory(path):
    ticks = []
    true_x = []
    true_y = []
    est_x = []
    est_y = []
    with open(path, "r") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            ticks.append(int(row["tick"]))
            true_x.append(float(row["true_x"]))
            true_y.append(float(row["true_y"]))
            est_x.append(float(row["est_x"]))
            est_y.append(float(row["est_y"]))
    return ticks, true_x, true_y, est_x, est_y


def style_axes(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for spine_name in ["top", "right"]:
        ax.spines[spine_name].set_visible(False)
    for spine_name in ["left", "bottom"]:
        ax.spines[spine_name].set_color(INK_MUTED)
    ax.tick_params(colors=INK_SECONDARY, labelsize=9)
    ax.xaxis.label.set_color(INK_SECONDARY)
    ax.yaxis.label.set_color(INK_SECONDARY)
    ax.title.set_color(INK)


def main():
    if not os.path.exists(FAULT_SWEEP_CSV):
        print("Missing " + FAULT_SWEEP_CSV + ". Run the fault-sweep mode first.")
        return
    if not os.path.exists(FAILOVER_LATENCIES_CSV):
        print("Missing " + FAILOVER_LATENCIES_CSV + ". Run comms_link/run_failover_trials.sh first.")
        return
    if not os.path.exists(TRAJECTORY_CSV):
        print("Missing " + TRAJECTORY_CSV + ". Run 'mission_computer sim samples/ekf_trajectory.csv' first.")
        return

    os.makedirs(ASSETS_DIR, exist_ok=True)

    correct, total = read_fault_sweep_success_rate(FAULT_SWEEP_CSV)
    success_rate = 100.0 * correct / total

    latencies = read_failover_latencies(FAILOVER_LATENCIES_CSV)

    ticks, true_x, true_y, est_x, est_y = read_trajectory(TRAJECTORY_CSV)

    figure, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    figure.patch.set_facecolor(SURFACE)

    # Panel 1: failover latency histogram.
    ax0 = axes[0]
    ax0.hist(latencies, bins=8, color=BLUE, edgecolor=SURFACE)
    ax0.axvline(250, color=ORANGE, linewidth=2, linestyle="--", label="250 ms target")
    ax0.set_title("comms_link failover latency\n(" + str(len(latencies)) + " simulated-outage trials)")
    ax0.set_xlabel("latency (ms, simulated time)")
    ax0.set_ylabel("trial count")
    ax0.legend(frameon=False, labelcolor=INK_SECONDARY, fontsize=9)
    style_axes(ax0)

    # Panel 2: safe-state transition success rate.
    ax1 = axes[1]
    bar_color = BLUE if success_rate >= 99.99 else ORANGE
    ax1.bar(["safe-state\ntransitions"], [success_rate], color=bar_color, width=0.5)
    ax1.set_ylim(0, 105)
    ax1.set_ylabel("success rate (%)")
    ax1.set_title("fault-sweep result\n(" + str(total) + " scenarios)")
    ax1.text(0, success_rate + 3, "{:.2f}%".format(success_rate), ha="center", color=INK, fontsize=11, fontweight="bold")
    style_axes(ax1)

    # Panel 3: EKF estimated vs true trajectory.
    ax2 = axes[2]
    ax2.plot(true_x, true_y, color=INK_MUTED, linewidth=2.5, label="true position")
    ax2.plot(est_x, est_y, color=ORANGE, linewidth=1.2, label="EKF estimate")
    ax2.set_title("EKF position estimate vs.\nsimulated true trajectory")
    ax2.set_xlabel("x (m)")
    ax2.set_ylabel("y (m)")
    ax2.legend(frameon=False, labelcolor=INK_SECONDARY, fontsize=9)
    ax2.set_aspect("equal", adjustable="datalim")
    style_axes(ax2)

    figure.suptitle(
        "Resilient Autonomous Mission Computer, simulation dashboard",
        color=INK, fontsize=13, fontweight="bold", y=1.03,
    )
    figure.tight_layout()
    figure.savefig(OUTPUT_PATH, dpi=150, bbox_inches="tight", facecolor=SURFACE)
    print("Wrote " + OUTPUT_PATH)
    print("Fault-sweep success rate: {:.2f}% ({}/{})".format(success_rate, correct, total))
    print("Failover latency samples (ms): " + str(latencies))


if __name__ == "__main__":
    main()
