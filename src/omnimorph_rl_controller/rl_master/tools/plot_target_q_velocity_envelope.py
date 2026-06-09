#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
os.environ.setdefault("XDG_CACHE_HOME", "/tmp")

import matplotlib.pyplot as plt
import numpy as np
import yaml


def clamp_target_q_to_velocity_envelope_delta(
    current_velocity: float,
    kp: float,
    kd: float,
    x1: float,
    x2: float,
    y1: float,
    y2: float,
    zero_velocity_epsilon: float,
) -> tuple[float, float, float, float]:
    if kp <= 0.0 or x2 <= x1 or y1 <= 0.0 or y2 <= 0.0:
        return 0.0, 0.0, 0.0, 0.0

    abs_velocity = abs(current_velocity)
    over_velocity = max(0.0, abs_velocity - x1)
    span = max(1.0e-6, x2 - x1)

    positive_base = y2 if abs_velocity <= zero_velocity_epsilon else (y1 if current_velocity >= 0.0 else y2)
    positive_slope = positive_base / span
    tau_high = max(0.0, positive_base - positive_slope * over_velocity)

    negative_base = -y2 if abs_velocity <= zero_velocity_epsilon else (-y2 if current_velocity >= 0.0 else -y1)
    negative_slope = (-negative_base) / span
    tau_low = min(0.0, negative_base + negative_slope * over_velocity)

    p_low = tau_low + kd * current_velocity
    p_high = tau_high + kd * current_velocity
    delta_q_low = p_low / kp
    delta_q_high = p_high / kp
    return tau_low, tau_high, delta_q_low, delta_q_high


def load_profile(profile_path: Path, profile_name: str) -> dict:
    with profile_path.open("r", encoding="utf-8") as f:
        root = yaml.safe_load(f)
    if not isinstance(root, dict) or profile_name not in root:
        raise RuntimeError(f"profile '{profile_name}' not found in {profile_path}")
    profile = root[profile_name]
    if not isinstance(profile, dict):
        raise RuntimeError(f"profile '{profile_name}' is not a map")
    return profile


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot target_q velocity envelope for one joint.")
    parser.add_argument("--profile", required=True, help="Path to profile yaml.")
    parser.add_argument("--profile-name", required=True, help="Top-level profile key.")
    parser.add_argument("--joint", required=True, help="Joint name to plot.")
    parser.add_argument("--output", required=True, help="Output PNG path.")
    parser.add_argument("--max-velocity", type=float, default=None, help="Absolute dq range. Default: 1.35 * x2.")
    parser.add_argument("--samples", type=int, default=1201, help="Number of dq samples.")
    args = parser.parse_args()

    profile = load_profile(Path(args.profile), args.profile_name)
    joint = args.joint

    kp = float(profile["kps"][joint])
    kd = float(profile["kds"][joint])
    env = profile["target_q_velocity_envelope"]
    x1 = float(env["x1"][joint])
    x2 = float(env["x2"][joint])
    y1 = float(env["y1"][joint])
    y2 = float(env["y2"][joint])
    zero_velocity_epsilon = float(env.get("zero_velocity_epsilon", 1.0e-2))

    max_velocity = args.max_velocity if args.max_velocity is not None else 1.35 * x2
    dq_values = np.linspace(-max_velocity, max_velocity, args.samples)
    tau_low = np.zeros_like(dq_values)
    tau_high = np.zeros_like(dq_values)
    delta_low = np.zeros_like(dq_values)
    delta_high = np.zeros_like(dq_values)

    for i, dq in enumerate(dq_values):
        t_low, t_high, q_low, q_high = clamp_target_q_to_velocity_envelope_delta(
            float(dq), kp, kd, x1, x2, y1, y2, zero_velocity_epsilon
        )
        tau_low[i] = t_low
        tau_high[i] = t_high
        delta_low[i] = q_low
        delta_high[i] = q_high

    neutral_delta = (kd * dq_values) / kp
    dq_x2_pos = x2
    dq_x2_neg = -x2
    _, _, delta_at_x2_neg, _ = clamp_target_q_to_velocity_envelope_delta(
        dq_x2_neg, kp, kd, x1, x2, y1, y2, zero_velocity_epsilon
    )
    _, _, _, delta_at_x2_pos = clamp_target_q_to_velocity_envelope_delta(
        dq_x2_pos, kp, kd, x1, x2, y1, y2, zero_velocity_epsilon
    )
    delta_at_zero = y2 / kp

    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True, constrained_layout=True)

    ax_tau, ax_q = axes
    ax_tau.fill_between(dq_values, tau_low, tau_high, color="#cfe8ff", alpha=0.85, label="allowed tau envelope")
    ax_tau.plot(dq_values, tau_low, color="#1f77b4", linewidth=1.8)
    ax_tau.plot(dq_values, tau_high, color="#1f77b4", linewidth=1.8)
    ax_tau.axvline(x1, color="#666666", linestyle="--", linewidth=1.0, label="X1 / X2")
    ax_tau.axvline(-x1, color="#666666", linestyle="--", linewidth=1.0)
    ax_tau.axvline(x2, color="#444444", linestyle=":", linewidth=1.2)
    ax_tau.axvline(-x2, color="#444444", linestyle=":", linewidth=1.2)
    ax_tau.axhline(0.0, color="#999999", linewidth=0.8)
    ax_tau.set_ylabel("Allowed tau (Nm)")
    ax_tau.set_title(
        f"OmniXtreme target_q velocity envelope: {joint}\n"
        f"kp={kp:.4f}, kd={kd:.4f}, X1={x1:.2f}, X2={x2:.2f}, Y1={y1:.2f}, Y2={y2:.2f}"
    )
    ax_tau.grid(True, alpha=0.25)
    ax_tau.legend(loc="upper right")

    ax_q.fill_between(dq_values, delta_low, delta_high, color="#d8f5d0", alpha=0.9, label="allowed target_q - current q")
    ax_q.plot(dq_values, delta_low, color="#2ca02c", linewidth=1.8)
    ax_q.plot(dq_values, delta_high, color="#2ca02c", linewidth=1.8)
    ax_q.plot(dq_values, neutral_delta, color="#d62728", linestyle="--", linewidth=1.2, label="kd * dq / kp")
    ax_q.axvline(x1, color="#666666", linestyle="--", linewidth=1.0)
    ax_q.axvline(-x1, color="#666666", linestyle="--", linewidth=1.0)
    ax_q.axvline(x2, color="#444444", linestyle=":", linewidth=1.2)
    ax_q.axvline(-x2, color="#444444", linestyle=":", linewidth=1.2)
    ax_q.axhline(0.0, color="#999999", linewidth=0.8)
    ax_q.scatter([0.0, dq_x2_neg, dq_x2_pos], [delta_at_zero, delta_at_x2_neg, delta_at_x2_pos], color="#111111", s=22)
    ax_q.annotate(f"+Y2/kp = {delta_at_zero:.3f} rad", xy=(0.0, delta_at_zero), xytext=(0.8, delta_at_zero + 0.12))
    ax_q.annotate(f"@ -X2: {delta_at_x2_neg:.3f} rad", xy=(dq_x2_neg, delta_at_x2_neg), xytext=(dq_x2_neg - 7.0, delta_at_x2_neg - 0.28))
    ax_q.annotate(f"@ +X2: {delta_at_x2_pos:.3f} rad", xy=(dq_x2_pos, delta_at_x2_pos), xytext=(dq_x2_pos - 6.0, delta_at_x2_pos + 0.15))
    ax_q.set_xlabel("Current joint velocity dq (rad/s)")
    ax_q.set_ylabel("Allowed target_q - q (rad)")
    ax_q.grid(True, alpha=0.25)
    ax_q.legend(loc="upper left")

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
