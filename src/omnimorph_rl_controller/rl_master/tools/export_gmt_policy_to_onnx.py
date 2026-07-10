#!/usr/bin/env python3
"""Export the official GMT TorchScript policy to the rl_master ONNX contract."""

from __future__ import annotations

import argparse
from pathlib import Path

import torch


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--opset", type=int, default=17)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    # PyTorch only uses onnx here to reparse the already generated protobuf and
    # inject custom functions. GMT has no custom ONNX functions, so exporting can
    # remain fully offline even in a Torch environment without the onnx package.
    try:
        import onnx  # noqa: F401
    except ImportError:
        from torch.onnx._internal import onnx_proto_utils

        onnx_proto_utils._add_onnxscript_fn = lambda model_bytes, custom_opsets: model_bytes

    model = torch.jit.load(str(args.input), map_location="cpu").eval()
    sample = torch.zeros(1, 2154, dtype=torch.float32)
    torch.onnx.export(
        model,
        sample,
        str(args.output),
        opset_version=args.opset,
        input_names=["obs"],
        output_names=["actions"],
        do_constant_folding=True,
    )
    print(f"exported: {args.output}")
    print("contract: obs[1,2154] -> actions[1,23]")


if __name__ == "__main__":
    main()
