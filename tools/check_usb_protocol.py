#!/usr/bin/env python3
"""Check fixed USB v4 byte vectors, optionally also against MCU protocol sources.

This checks the selected wire examples, not source identity or complete firmware
behavior. MCU checkout is read-only; compilation happens in a temporary folder.
"""
import argparse
import difflib
import os
import re
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile


def check_vectors(label, sources, includes, defines, compiler, probe, expected, work):
    executable = work / label
    command = compiler + ["-std=c++20", str(probe)]
    command += [str(source) for source in sources]
    command += [f"-I{directory}" for directory in includes]
    command += defines + ["-o", str(executable)]
    subprocess.run(command, check=True, timeout=60)
    actual = subprocess.check_output([str(executable)], text=True, timeout=10)
    if actual != expected:
        sys.stderr.writelines(difflib.unified_diff(
            expected.splitlines(keepends=True), actual.splitlines(keepends=True),
            fromfile="USB v4 baseline", tofile=label))
        return False
    print(f"{label}: {len(expected.splitlines())} byte vectors/signatures matched")
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mcu_checkout", type=Path, nargs="?",
                        help="also compile vectors against MCU src/ protocol sources")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    compiler = shlex.split(os.environ.get("CXX", "c++"))
    probe = root / "tests/protocols/usb_wire_vectors.cpp"
    expected = (root / "tests/protocols/usb_v4_vectors.txt").read_text()
    try:
        with tempfile.TemporaryDirectory(prefix="dust-usb-wire-") as temporary:
            work = Path(temporary)
            if not check_vectors(
                "host", [root / "src/hardware/usb/protocol/usb_protocol.cpp"],
                [root / "src"], [], compiler, probe, expected, work
            ):
                return 1
            if args.mcu_checkout is not None:
                candidates = [args.mcu_checkout / "src/hardware/usb/protocol",
                              args.mcu_checkout / "src"]
                directory = next((path for path in candidates
                                  if (path / "usb_protocol.hpp").is_file()), None)
                if directory is None:
                    parser.error("MCU usb_protocol.hpp not found under src/ or src/hardware/usb/protocol/")
                defines = ['-DDUST_USB_PROTOCOL_HEADER="usb_protocol.hpp"']
                # 两端源码命名空间可独立演进；比较的是同一组线上字节。
                header = (directory / "usb_protocol.hpp").read_text()
                if re.search(r"namespace\s+dust\s*::\s*wire\s*\{", header):
                    defines += ["-DDUST_USB_WIRE_NAMESPACE=dust::wire"]
                # 兼容旧的头文件实现，也允许 MCU 独立维护 hpp/cpp 实现。
                implementation = directory / "usb_protocol.cpp"
                sources = [implementation] if implementation.is_file() else []
                if (directory / "usb_session_policy.hpp").is_file():
                    defines += ['-DDUST_USB_LEGACY_SESSION_HEADER="usb_session_policy.hpp"']
                if not check_vectors("mcu", sources, [directory], defines,
                                     compiler, probe, expected, work):
                    return 1
    except (OSError, subprocess.SubprocessError) as error:
        print(f"USB protocol check failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
