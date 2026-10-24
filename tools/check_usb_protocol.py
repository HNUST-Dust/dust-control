#!/usr/bin/env python3
"""Check portable USB protocol sources against a dust-adapter checkout."""
import argparse
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mcu_checkout", type=Path)
    args = parser.parse_args()
    host = Path(__file__).resolve().parents[1] / "src/hardware/usb/protocol"
    mcu = args.mcu_checkout / "src"
    failed = False
    for name in ("usb_protocol.hpp", "usb_protocol_codec.hpp", "usb_session_policy.hpp"):
        left, right = host / name, mcu / name
        if not right.is_file():
            print(f"Missing MCU file: {right}", file=sys.stderr)
            failed = True
        elif left.read_bytes() != right.read_bytes():
            print(f"Protocol source differs: {name}", file=sys.stderr)
            failed = True
        else:
            print(f"Identical: {name}")
    return int(failed)


if __name__ == "__main__":
    sys.exit(main())
