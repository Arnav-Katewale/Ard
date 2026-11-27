"""Fixed vs adaptive gait on the MuJoCo terrains.

The same control loop runs in four modes per terrain:

    fixed               the config with adaptation switched off (fixed_gait_config):
                        body-correction PID gains 0, every stride and cycle-time
                        scale 1, no extra step height: always the base gait
    fixed_half_stride   fixed, but at half the base stride
    adaptive_no_balance the config as it is, but body-correction PID gains 0
    adaptive            config/robot.toml as it is

The two middle modes separate the adaptive controller's two effects. While
walking, the placeholder stability references make the monitor report
UNSTABLE, which halves the stride; the PIDs tilt the body against the
measured tilt. fixed_half_stride has only the first, adaptive_no_balance has
the stride changes without the second.

Both modes keep the safety layer: obstacle STOP still brings the robot to a
stand, and the tilt, IMU-failure and rejection emergency stops still fire.

Everything here is simulation with placeholder physics. A result says how the
two modes compare in this model, not on the real robot.
"""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass
from typing import Any

from robodog.mujoco_sim.backends import MujocoSim
from robodog.mujoco_sim.runner import build_sim_control_loop, run

MODES = ("fixed", "fixed_half_stride", "adaptive_no_balance", "adaptive")


def _no_balance(cfg: dict[str, Any]) -> dict[str, Any]:
    out = copy.deepcopy(cfg)
    for pid in ("roll_pid", "pitch_pid"):
        out["adaptive"][pid]["kp"] = out["adaptive"][pid]["ki"] = out["adaptive"][pid]["kd"] = 0.0
    return out


def fixed_gait_config(cfg: dict[str, Any], stride_scale: float = 1.0) -> dict[str, Any]:
    """A copy of `cfg` whose adaptive controller always walks the base gait
    (with its stride scaled by `stride_scale`)."""
    out = _no_balance(cfg)
    a = out["adaptive"]
    for table in ("obstacle_stride_scale", "obstacle_cycle_time_scale"):
        a[table] = {level: 1.0 for level in a[table]}
    a["stability_stride_scale"] = {level: stride_scale for level in a["stability_stride_scale"]}
    a["obstacle_step_height_add_mm"] = {level: 0.0 for level in a["obstacle_step_height_add_mm"]}
    return out


def mode_config(cfg: dict[str, Any], mode: str) -> dict[str, Any]:
    """The config a mode runs with (see the module docstring)."""
    if mode == "fixed":
        return fixed_gait_config(cfg)
    if mode == "fixed_half_stride":
        return fixed_gait_config(cfg, stride_scale=0.5)
    if mode == "adaptive_no_balance":
        return _no_balance(cfg)
    if mode == "adaptive":
        return cfg
    raise ValueError(f"mode must be one of {MODES}, got {mode!r}")


@dataclass(frozen=True)
class Sample:
    t_s: float
    true_pitch_deg: float
    true_roll_deg: float
    imu_pitch_deg: float
    imu_roll_deg: float
    x_m: float
    stride_mm: float


@dataclass(frozen=True)
class Trial:
    terrain: str
    mode: str
    samples: list[Sample]
    estop_reason: str | None


@dataclass(frozen=True)
class Summary:
    rms_tilt_deg: float      # RMS over samples of the larger of |pitch| and |roll| (true values)
    peak_tilt_deg: float
    distance_m: float        # forward (x) progress from the first to the last sample
    estop_reason: str | None


def run_trial(cfg: dict[str, Any], terrain: str, mode: str, seconds: float) -> Trial:
    """Walk `seconds` on `terrain`; stop early at an emergency stop."""
    run_cfg = mode_config(cfg, mode)
    sim = MujocoSim(run_cfg, terrain=terrain)
    loop = build_sim_control_loop(run_cfg, sim)
    samples, estop = [], None
    for result in run(loop, sim, seconds):
        if result.estop_reason:
            estop = result.estop_reason
            break
        pitch, roll = sim.body_pitch_roll_deg()
        samples.append(Sample(
            t_s=sim.clock(), true_pitch_deg=pitch, true_roll_deg=roll,
            imu_pitch_deg=result.imu.pitch_deg, imu_roll_deg=result.imu.roll_deg,
            x_m=sim.torso_position_m()[0], stride_mm=result.adaptive.params.stride_length_mm,
        ))
    return Trial(terrain, mode, samples, estop)


def summarise(trial: Trial) -> Summary:
    s = trial.samples
    if not s:
        return Summary(math.nan, math.nan, 0.0, trial.estop_reason)
    tilts = [max(abs(x.true_pitch_deg), abs(x.true_roll_deg)) for x in s]
    return Summary(
        rms_tilt_deg=math.sqrt(sum(t * t for t in tilts) / len(tilts)),
        peak_tilt_deg=max(tilts),
        distance_m=s[-1].x_m - s[0].x_m,
        estop_reason=trial.estop_reason,
    )


def plot_trials(trials: list[Trial], path: str) -> None:
    """One row per terrain: true pitch and roll over time, fixed vs adaptive
    (the other modes are in the printed table only, to keep the plot readable)."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    terrains = list(dict.fromkeys(t.terrain for t in trials))
    fig, axes = plt.subplots(len(terrains), 2, figsize=(10, 2.4 * len(terrains)),
                             sharex=True, squeeze=False)
    colours = {"fixed": "#d1495b", "adaptive": "#2e86ab"}
    for row, terrain in enumerate(terrains):
        for col, (field, label) in enumerate((("true_pitch_deg", "pitch"), ("true_roll_deg", "roll"))):
            ax = axes[row][col]
            for t in trials:
                if t.terrain == terrain and t.mode in colours:
                    ax.plot([s.t_s for s in t.samples], [getattr(s, field) for s in t.samples],
                            color=colours[t.mode], lw=1, label=t.mode)
            ax.set_title(f"{terrain}: {label}", fontsize=9)
            ax.set_ylabel("deg")
            ax.grid(alpha=0.3)
    axes[0][0].legend(fontsize=8)
    for ax in axes[-1]:
        ax.set_xlabel("simulated time (s)")
    fig.suptitle("Fixed vs adaptive gait, MuJoCo simulation (placeholder physics)", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
