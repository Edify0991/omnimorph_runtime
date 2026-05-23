#!/usr/bin/env python3
from __future__ import annotations

import argparse
from typing import Optional, Sequence

import evdev
from evdev import ecodes


def find_joystick_device(keywords: Sequence[str]) -> Optional[evdev.InputDevice]:
    for path in evdev.list_devices():
        device = evdev.InputDevice(path)
        if any(token in device.name for token in keywords):
            return device
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Print raw evdev ABS/KEY events for a joystick")
    parser.add_argument("--joystick-keywords", default="X-Box,Xbox")
    args = parser.parse_args()

    keywords = tuple(k.strip() for k in args.joystick_keywords.split(",") if k.strip())
    device = find_joystick_device(keywords)
    if device is None:
        print("[JOY][ERR] joystick not found")
        return 1

    print(f"[JOY] device: {device.path} ({device.name})")
    print("[JOY] move sticks / triggers / dpad, press Ctrl+C to stop")

    try:
        for event in device.read_loop():
            if event.type == ecodes.EV_ABS:
                code_name = ecodes.bytype[ecodes.EV_ABS].get(event.code, str(event.code))
                print(f"ABS {code_name:<10} value={event.value}", flush=True)
            elif event.type == ecodes.EV_KEY:
                code_name = ecodes.bytype[ecodes.EV_KEY].get(event.code, str(event.code))
                print(f"KEY {code_name:<16} value={event.value}", flush=True)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
