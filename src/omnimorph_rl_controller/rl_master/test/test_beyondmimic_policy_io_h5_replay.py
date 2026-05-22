#!/usr/bin/env python3
"""Replay BeyondMimic policy_io H5 observations through the deploy ONNX policy."""

from __future__ import annotations

import subprocess
import sys
import unittest
from pathlib import Path


class BeyondMimicPolicyIoReplayTest(unittest.TestCase):
    def test_jc01_walk2_policy_io_replay_matches_recorded_actions(self) -> None:
        repo_root = Path(__file__).resolve().parents[4]
        tool = (
            repo_root
            / "src"
            / "omnimorph_rl_controller"
            / "rl_master"
            / "tools"
            / "analysis"
            / "compare_policy_io_h5.py"
        )
        h5_path = Path("/home/edify/Code/beyondmimic/outputs/policy_io/jc01_walk2_policy_io.h5")
        onnx_path = (
            repo_root
            / "src"
            / "omnimorph_rl_controller"
            / "rl_master"
            / "policies"
            / "2026-03-29_beyondmimic_jc01_leg12_strict_walk2.onnx"
        )

        if not h5_path.exists():
            self.skipTest(f"recorded policy_io H5 not found: {h5_path}")
        if not onnx_path.exists():
            self.skipTest(f"deploy ONNX policy not found: {onnx_path}")

        result = subprocess.run(
            [
                sys.executable,
                str(tool),
                "--h5",
                str(h5_path),
                "--mae-threshold",
                "1e-5",
                "--max-threshold",
                "1e-4",
            ],
            cwd=str(repo_root),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stdout)


if __name__ == "__main__":
    unittest.main()
