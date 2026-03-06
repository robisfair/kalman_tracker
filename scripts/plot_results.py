#!/usr/bin/env python3
"""
plot_results.py — Visualize Kalman Filter tracking results from CSV output.

Usage:
    python3 scripts/plot_results.py data/output_linear_3sensor.csv
"""

import sys
import csv
import math
import os
try:
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
except ImportError:
    print("Install matplotlib:  pip install matplotlib")
    sys.exit(1)

def load_csv(path):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: float(v) for k, v in row.items()})
    return rows

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <output.csv>")
        sys.exit(1)

    path = sys.argv[1]
    if not os.path.exists(path):
        print(f"File not found: {path}")
        sys.exit(1)

    data  = load_csv(path)
    times = [r["time"]     for r in data]
    tx    = [r["truth_x"]  for r in data]
    ty    = [r["truth_y"]  for r in data]
    ex    = [r["est_x"]    for r in data]
    ey    = [r["est_y"]    for r in data]
    px    = [r["prior_x"]  for r in data]
    py    = [r["prior_y"]  for r in data]
    inn_x = [r["innovation_x"] for r in data]
    inn_y = [r["innovation_y"] for r in data]

    pos_err = [math.sqrt((ex[i]-tx[i])**2 + (ey[i]-ty[i])**2)
               for i in range(len(data))]

    scenario = os.path.splitext(os.path.basename(path))[0].replace("output_", "")

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(f"Kalman Filter Tracking — {scenario}", fontsize=14, fontweight="bold")
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.4, wspace=0.35)

    # ── 1. XY trajectory ──
    ax1 = fig.add_subplot(gs[0, :2])
    ax1.plot(tx, ty, "k-",  linewidth=2,   label="Ground Truth")
    ax1.plot(ex, ey, "b--", linewidth=1.5, label="KF Estimate")
    ax1.plot(px, py, "r:",  linewidth=1,   alpha=0.5, label="KF Prior (predicted)")
    ax1.scatter(tx[0], ty[0], c="green", s=80, zorder=5, label="Start")
    ax1.scatter(tx[-1], ty[-1], c="red",  s=80, zorder=5, marker="X", label="End")
    ax1.set_xlabel("X Position (m)")
    ax1.set_ylabel("Y Position (m)")
    ax1.set_title("Target Trajectory")
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    # ── 2. Position error over time ──
    ax2 = fig.add_subplot(gs[0, 2])
    ax2.plot(times, pos_err, "b-", linewidth=1)
    ax2.axhline(y=sum(pos_err)/len(pos_err), color="r", linestyle="--", label=f"Mean={sum(pos_err)/len(pos_err):.1f}m")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Position Error (m)")
    ax2.set_title("KF Position Error")
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    # ── 3. X component over time ──
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(times, tx, "k-",  linewidth=2,   label="Truth X")
    ax3.plot(times, ex, "b--", linewidth=1.5, label="Est X")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("X Position (m)")
    ax3.set_title("X Position vs Time")
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)

    # ── 4. Y component over time ──
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(times, ty, "k-",  linewidth=2,   label="Truth Y")
    ax4.plot(times, ey, "b--", linewidth=1.5, label="Est Y")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Y Position (m)")
    ax4.set_title("Y Position vs Time")
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)

    # ── 5. Innovations ──
    ax5 = fig.add_subplot(gs[1, 2])
    ax5.plot(times, inn_x, "b-", alpha=0.8, label="Innovation X")
    ax5.plot(times, inn_y, "r-", alpha=0.8, label="Innovation Y")
    ax5.axhline(0, color="k", linewidth=0.5)
    ax5.set_xlabel("Time (s)")
    ax5.set_ylabel("Innovation (m)")
    ax5.set_title("Filter Innovations (z - Hx̂)")
    ax5.legend(fontsize=8)
    ax5.grid(True, alpha=0.3)

    out_path = path.replace(".csv", "_plot.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"Saved plot: {out_path}")
    plt.show()

if __name__ == "__main__":
    main()
