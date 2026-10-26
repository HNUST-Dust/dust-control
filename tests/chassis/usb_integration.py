"""Real production host lifecycle, with a test-only USB backend and no hardware."""
import os
import subprocess
import sys
from pathlib import Path
import tempfile
import shutil
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from startup_support import configuration, node_command, run_for

binary, backend = sys.argv[1:]
for mode in (
    None,
    "WBR_MOCK_BAD_CAPS",
    "WBR_MOCK_OLD_FIRMWARE",
    "WBR_MOCK_DISCONNECT",
    "WBR_MOCK_SAFETY_REJECT",
    "WBR_MOCK_ARM_REJECT",
    "WBR_MOCK_V1_FIRMWARE",
    "WBR_MOCK_DATA_REJECT",
    "WBR_MOCK_PROFILE_MISMATCH",
    "WBR_MOCK_NO_PROFILE",
):
    environment = dict(os.environ, LD_PRELOAD=backend, WBR_MOCK_REQUIRE_PREPOST="1")
    if mode:
        environment[mode] = "1"
    with configuration(simulate=False) as config:
        result = run_for(node_command(binary, config), env=environment, seconds=0.4)
    expected = 0 if mode in (None, "WBR_MOCK_PROFILE_MISMATCH") else 1
    assert result.returncode == expected, (
        mode,
        result.stdout,
        result.stderr,
        result.returncode,
    )
    if mode is None:
        assert "output=disabled" in result.stdout and "Stopped;" in result.stdout
    elif mode == "WBR_MOCK_OLD_FIRMWARE":
        assert "handshake timed out" in result.stderr
    elif mode == "WBR_MOCK_BAD_CAPS":
        assert "capability missing" in result.stderr
    elif mode in ("WBR_MOCK_SAFETY_REJECT", "WBR_MOCK_ARM_REJECT"):
        assert "rejected safety" in result.stderr
    elif mode == "WBR_MOCK_V1_FIRMWARE":
        assert "version mismatch" in result.stderr
    elif mode == "WBR_MOCK_PROFILE_MISMATCH":
        assert "Stopped;" in result.stdout
    elif mode == "WBR_MOCK_NO_PROFILE":
        assert "profile missing" in result.stderr
    elif mode == "WBR_MOCK_DATA_REJECT":
        assert "rejected a data request" in result.stderr
    else:
        assert (
            "USB IN status=5 actual_length=0 invalid report length" in result.stderr
        ), (mode, result.stdout, result.stderr)
print(
    "Production handshake, merged commands, sensor reception, stop, old firmware rejection and disconnect passed"
)

serial0 = "00112233445566778899AABBCCDDEEFF0"
serial1 = "00112233445566778899AABBCCDDEEFF1"
cases = [
    ({}, {}, 0, "PID=34b7:1234"),
    ({"WBR_MOCK_ZERO_PID": "1"}, {"pid": 0}, 0, "PID=34b7:0000"),
    ({"WBR_MOCK_LEGACY_PID": "1"}, {}, 0, "PID=34b7:ffff"),
    ({}, {"pid": 0xFFFF}, 1, "No matching"),
    ({}, {"serial": serial0}, 0, "serial=" + serial0),
    ({}, {"serial": "001122"}, 1, "No matching"),
    ({"WBR_MOCK_MULTIPLE": "1"}, {}, 1, "Multiple matching"),
    ({"WBR_MOCK_MULTIPLE": "1", "WBR_MOCK_SAME_PID": "1"}, {}, 1, "Multiple matching"),
    (
        {"WBR_MOCK_MULTIPLE": "1", "WBR_MOCK_SAME_PID": "1"},
        {"serial": serial1},
        0,
        "serial=" + serial1,
    ),
    ({"WBR_MOCK_MULTIPLE": "1"}, {"pid": 0xABCD}, 0, "serial=" + serial1),
    ({"WBR_MOCK_NO_DEVICE": "1"}, {}, 1, "No matching"),
    ({"WBR_MOCK_NO_ACCESS": "1"}, {}, 1, "LIBUSB_ERROR_ACCESS"),
    (
        {"WBR_MOCK_MULTIPLE": "1", "WBR_MOCK_SECOND_NO_ACCESS": "1"},
        {},
        1,
        "discovery incomplete",
    ),
    ({"WBR_MOCK_STRING_FAIL": "1"}, {}, 1, "manufacturer: LIBUSB_ERROR_IO"),
    ({"WBR_MOCK_BAD_INTERFACE": "1"}, {}, 1, "vendor interface 0"),
    ({"WBR_MOCK_OTHER_VENDOR": "1"}, {}, 1, "No matching"),
    ({"WBR_MOCK_OTHER_BOARD": "1"}, {}, 1, "No matching"),
    ({"WBR_MOCK_OTHER_BOARD": "1"}, {"product": "dust-other"}, 0, "product=dust-other"),
]
with tempfile.TemporaryDirectory() as directory:
    config = Path(directory)
    for source in (Path(__file__).resolve().parents[2] / "config").glob("*.yaml"):
        shutil.copy(source, config / source.name)
    original = yaml.safe_load((config / "dust_control.yaml").read_text())
    original["execution"] = {
        "simulate": False,
        "realtime": False,
        "enable_output": False,
    }
    for settings, identity, code, message in cases:
        document = yaml.safe_load(yaml.safe_dump(original))
        adapter = document["adapters"]["base"]
        adapter.update(identity)
        (config / "dust_control.yaml").write_text(yaml.safe_dump(document))
        environment = dict(os.environ, LD_PRELOAD=backend, **settings)
        # 失败用例等待自然退出；不要在 ROS 初始化尚未完成时发送 SIGINT。
        if code:
            result = subprocess.run(node_command(binary, config), env=environment,
                                    capture_output=True, text=True, timeout=6)
        else:
            result = run_for(node_command(binary, config), env=environment, seconds=0.3)
        assert result.returncode == code, (
            settings,
            identity,
            result.returncode,
            result.stdout,
            result.stderr,
        )
        assert message in result.stderr, (settings, identity, result.stderr)
print(
    "USB identity discovery, ambiguity, YAML serial/PID selection and diagnostics passed"
)
