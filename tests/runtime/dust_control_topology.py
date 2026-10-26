"""YAML composition and identity validation; all USB goes to the mock backend."""
import copy
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from startup_support import node_command, run_for

fixture, backend, config_path, controller = sys.argv[1:]
config_path = Path(config_path)
original = yaml.safe_load((config_path / "dust_control.yaml").read_text())
serial0 = "00112233445566778899AABBCCDDEEFF0"
serial1 = "00112233445566778899AABBCCDDEEFF1"
cpu = min(os.sched_getaffinity(0))


def run(binary, config, *, expected=0, environment=None, arguments=()):
    if binary == fixture:
        command = [binary, *arguments, str(config)]
    elif binary:
        command = node_command(binary, config) + list(arguments)
    else:
        command = [fixture, "--validate", str(config)]
    if not binary or binary == fixture or expected:
        result = subprocess.run(
            command, env=environment, capture_output=True, text=True, timeout=6
        )
    else:
        result = run_for(command, env=environment)
    assert result.returncode == expected, (
        result.returncode,
        result.stdout,
        result.stderr,
    )
    return result


with tempfile.TemporaryDirectory() as directory:
    config = Path(directory)
    for source in config_path.glob("*.yaml"):
        shutil.copy(source, config / source.name)
    document = copy.deepcopy(original)
    drive = document["adapters"].pop("base")
    drive["serial"] = serial0
    drive["receive_thread"] = {"cpu": cpu, "priority": 0}
    drive["send_thread"] = {"cpu": cpu, "priority": 0}
    head = {
        "serial": serial1,
        "receive_thread": {"cpu": None, "priority": 0},
        "send_thread": {"cpu": None, "priority": 0},
    }
    document["adapters"] = {"drive": drive, "head": head}
    for module in document["modules"].values():
        module["thread"] = {"cpu": cpu, "priority": 0}
    document["modules"]["chassis"]["sensor_adapter"] = "drive"
    document["modules"]["ui"]["adapter"] = "drive"
    document["modules"]["gimbal"]["adapter"] = "head"
    original_motors = yaml.safe_load((config_path / original["motors"]).read_text())
    for device in original_motors["motors"]:
        device["adapter"] = "drive"
    (config / document["motors"]).write_text(yaml.safe_dump(original_motors))
    (config / "dust_control.yaml").write_text(yaml.safe_dump(document))
    environment = dict(
        os.environ,
        LD_PRELOAD=backend,
        WBR_MOCK_MULTIPLE="1",
        WBR_MOCK_REQUIRE_PREPOST="1",
    )
    result = run(fixture, config, environment=environment)
    assert result.stderr.count("USB bridge bus=") == 1
    assert "Adapter head: configured, idle" in result.stdout
    assert "Two named adapters" in result.stdout

    # 任意有效绑定、禁用和缺省兼容均应通过配置加载。
    for bindings in ({"remote": 4, "hi91": 6, "referee": 5},
                     {"remote": None, "hi91": None, "referee": None}, None):
        valid = copy.deepcopy(document)
        if bindings is None:
            del valid["adapters"]["drive"]["uart_sensors"]
        else:
            valid["adapters"]["drive"]["uart_sensors"] = bindings
        (config / "dust_control.yaml").write_text(yaml.safe_dump(valid))
        run("", config, environment=environment)

    # 配置错误必须在 USB 打开、线程启动之前拒绝。
    for case in (
        "missing",
        "version",
        "unknown",
        "duplicate",
        "reference",
        "cpu",
        "priority",
        "serial",
        "automatic",
        "name",
        "motors",
        "all_disabled",
        "uart_duplicate",
        "uart_range",
        "uart_zero",
        "uart_unknown",
        "uart_missing",
    ):
        invalid = copy.deepcopy(document)
        if case.startswith("uart_"):
            binding = invalid["adapters"]["drive"]["uart_sensors"]
            if case == "uart_duplicate": binding["hi91"] = binding["remote"]
            elif case == "uart_range": binding["hi91"] = 7
            elif case == "uart_zero": binding["hi91"] = 0
            elif case == "uart_unknown": binding["other"] = 4
            elif case == "uart_missing": del binding["hi91"]
        elif case == "version":
            invalid["schema_version"] = 99
        elif case == "unknown":
            invalid["modules"]["chassis"]["unknown"] = True
        elif case == "reference":
            invalid["modules"]["ui"]["adapter"] = "absent"
        elif case == "cpu":
            invalid["modules"]["gimbal"]["thread"]["cpu"] = -2
        elif case == "priority":
            invalid["adapters"]["drive"]["send_thread"]["priority"] = 100
        elif case == "serial":
            invalid["adapters"]["head"]["serial"] = serial0
        elif case == "automatic":
            invalid["adapters"]["head"]["serial"] = ""
        elif case == "name":
            invalid["adapters"]["too_long_name"] = invalid["adapters"].pop("head")
        elif case == "motors":
            invalid["motors"] = "missing.yaml"
        elif case == "all_disabled":
            for module in invalid["modules"].values():
                module["enabled"] = False
        (config / "dust_control.yaml").write_text(yaml.safe_dump(invalid))
        if case == "missing":
            (config / "dust_control.yaml").unlink()
        elif case == "duplicate":
            with (config / "dust_control.yaml").open("a") as stream:
                stream.write("modules: {}\n")
        result = run(controller, config, expected=1, environment=environment)
        assert "USB bridge bus=" not in result.stderr
        assert "Chassis: 1 kHz" not in result.stdout

    # Adapter 归属只来自每台电机，模块不再声明 motor_adapters。
    for count in (2, 3):
        split = copy.deepcopy(document)
        if count == 3:
            split["adapters"]["aux"] = copy.deepcopy(head)
            split["adapters"]["aux"]["serial"] = "00112233445566778899AABBCCDDEEFF2"
        motors = copy.deepcopy(original_motors)
        for device in motors["motors"]:
            role = device["name"]
            device["adapter"] = "drive" if "wheel" in role else "head"
            if count == 3 and role.startswith("right_joint"):
                device["adapter"] = "aux"
                device["bus"] = 3
                device["command_id"] = 0 if role.endswith("b") else 3
                device["feedback_ids"] = [0x10 + device["command_id"]]
        (config / split["motors"]).write_text(yaml.safe_dump(motors))
        (config / "dust_control.yaml").write_text(yaml.safe_dump(split))
        split_environment = dict(environment, WBR_MOCK_TRIPLE="1", WBR_MOCK_SPLIT=str(count))
        result = run(fixture, config, environment=split_environment, arguments=("--split",))
        assert result.stderr.count("USB bridge bus=") == count
        assert "Split motor routing and feedback passed" in result.stdout
        if count == 3:
            delayed = dict(split_environment, WBR_MOCK_DELAY_INDEX="1")
            run(fixture, config, environment=delayed, arguments=("--split",))
            failing = dict(split_environment, WBR_MOCK_ARM_REJECT_INDEX="1")
            failed = run(fixture, config, expected=1, environment=failing, arguments=("--split",))
            assert "Adapter aux: Bridge capabilities" in failed.stdout
            assert "Adapter drive: Bridge capabilities" in failed.stdout
            assert "Adapter head:" in failed.stderr
            disconnected = dict(split_environment, WBR_MOCK_DISCONNECT_INDEX="2")
            run(fixture, config, environment=disconnected, arguments=("--split",))
            for case in ("missing", "duplicate", "unknown_adapter", "same_adapter_collision", "old_binding", "owner_missing"):
                invalid = copy.deepcopy(split)
                invalid_motors = copy.deepcopy(motors)
                if case == "missing":
                    invalid_motors["motors"].pop()
                elif case == "duplicate":
                    invalid_motors["motors"].append(copy.deepcopy(invalid_motors["motors"][0]))
                elif case == "unknown_adapter":
                    invalid_motors["motors"][0]["adapter"] = "unknown"
                elif case == "same_adapter_collision":
                    # left/right 腿的 bus/ID 相同：归属不同板合法，改成同板必须拒绝。
                    for device in invalid_motors["motors"]:
                        if device["adapter"] == "aux":
                            device["adapter"] = "head"
                elif case == "old_binding":
                    invalid["modules"]["chassis"]["motor_adapters"] = ["drive", "head", "aux"]
                elif case == "owner_missing":
                    del invalid_motors["motors"][0]["adapter"]
                (config / split["motors"]).write_text(yaml.safe_dump(invalid_motors))
                (config / "dust_control.yaml").write_text(yaml.safe_dump(invalid))
                result = run("", config, expected=1, environment=split_environment)
                assert "USB bridge bus=" not in result.stderr

    # 禁用底盘时仍可通过信号停止空模块。
    only_ui = copy.deepcopy(original)
    only_ui["execution"] = {"simulate": True, "realtime": False, "enable_output": False}
    only_ui["modules"]["chassis"]["enabled"] = False
    only_ui["modules"]["gimbal"]["enabled"] = False
    only_ui["modules"]["ui"]["adapter"] = None
    (config / "dust_control.yaml").write_text(yaml.safe_dump(only_ui))
    result = run(controller, config, environment=environment)
    assert "Chassis: 1 kHz" not in result.stdout
    assert "USB bridge bus=" not in result.stderr

print(
    "DustControl YAML, named adapters, idle sessions, CPU assignment, disabled modules and startup rejection passed"
)
