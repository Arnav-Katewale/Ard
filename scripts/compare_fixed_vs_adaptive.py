"""Run fixed vs adaptive gait on each MuJoCo terrain; print a table and save a plot.

    pip install -r requirements-sim.txt
    python scripts/compare_fixed_vs_adaptive.py [--seconds 8] [--out docs/sim]

Simulation only, placeholder physics: see robodog/mujoco_sim/compare.py.
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from robodog.config import load_config  # noqa: E402
from robodog.mujoco_sim.compare import MODES, plot_trials, run_trial, summarise  # noqa: E402

TERRAINS = ("flat", "side_slope", "ramp", "bumps")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--out", default="docs/sim")
    args = parser.parse_args()

    cfg = load_config()
    trials = [run_trial(cfg, terrain, mode, args.seconds) for terrain in TERRAINS for mode in MODES]

    print(f"| Terrain | Mode | RMS tilt (deg) | Peak tilt (deg) | Distance (m) | E-stop |")
    print(f"|---|---|---|---|---|---|")
    for t in trials:
        s = summarise(t)
        print(f"| {t.terrain} | {t.mode} | {s.rms_tilt_deg:.1f} | {s.peak_tilt_deg:.1f} | "
              f"{s.distance_m:.2f} | {s.estop_reason or 'none'} |")

    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, "fixed_vs_adaptive.png")
    plot_trials(trials, path)
    print(f"plot: {path}")


if __name__ == "__main__":
    main()
