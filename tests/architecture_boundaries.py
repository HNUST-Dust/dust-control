"""Keep the controller, device drivers and transport independently usable."""
from pathlib import Path
import re
import sys

root = Path(sys.argv[1])
violations = []
rules = {
    "src/chassis": ("hardware/", "runtime/"),
    "src/gimbal": ("chassis/runtime/chassis_runtime.hpp", "ui/ui_runtime.hpp"),
    "src/ui": ("chassis/runtime/chassis_runtime.hpp", "gimbal/gimbal_runtime.hpp"),
    "src/hardware/ethercat": ("chassis/",),
    **{f"src/hardware/{group}": ("hardware/ethercat/", "chassis/", "runtime/")
       for group in ("motors", "remote", "imu")},
}
# 底盘只引用设备侧的被动数据定义，不能引用 Decoder、Encoder 或通信实现。
chassis_data_headers = {
    "hardware/remote/remote_input_state.hpp",
    "hardware/motors/motor_types.hpp",
    "hardware/imu/imu_sample.hpp",
}
for directory, forbidden in rules.items():
    for path in (root / directory).rglob("*"):
        if path.suffix not in (".cpp", ".cc", ".h", ".hpp"):
            continue
        for number, line in enumerate(path.read_text().splitlines(), 1):
            match = re.match(r'\s*#\s*include\s*[<"]([^>"]+)', line)
            if (directory in ("src/gimbal", "src/ui") and match and
                    match[1] in ("dust_control.hpp", "dust_control_config.hpp", "hardware/ethercat/master.hpp", "hardware/ethercat/session.hpp", "hardware/ethercat/transport.hpp")):
                violations.append(f"{path.relative_to(root)}:{number}: business module owns startup/EtherCAT lifecycle")
            is_chassis_runtime = directory == "src/chassis" and path.name in ("chassis_runtime.cpp", "chassis_runtime.hpp", "chassis_config_loading.cpp")
            if is_chassis_runtime and path.name != "chassis_config_loading.cpp" and match and match[1] in (
                "dust_control.hpp", "dust_control_config.hpp", "hardware/ethercat/master.hpp", "hardware/ethercat/session.hpp", "hardware/ethercat/transport.hpp",
                "yaml_validation.hpp",
            ):
                violations.append(f"{path.relative_to(root)}:{number}: chassis runtime owns startup dependency")
            if (directory == "src/chassis" and not is_chassis_runtime and match
                    and match[1].endswith("chassis/runtime/chassis_runtime.hpp")):
                violations.append(f"{path.relative_to(root)}:{number}: algorithm includes business runtime")
            if (not is_chassis_runtime and match and match[1].startswith(forbidden)
                    and not (directory == "src/chassis" and match[1] in chassis_data_headers)):
                violations.append(f"{path.relative_to(root)}:{number}: {match[1]}")
            if path.relative_to(root).as_posix().removeprefix("src/") in chassis_data_headers:
                if ((match and match[1] not in ("array", "cstdint"))
                        or "protocols::" in line):
                    violations.append(f"{path.relative_to(root)}:{number}: data header depends on implementation")
            # 数据定义独立于协议服务；算法不能使用设备服务。
            if directory == "src/chassis" and not is_chassis_runtime and re.search(
                    r"\b(MotorEncoder|MotorCommandEncoder|MotorFeedbackDecoder|DecodeMotorFeedback|WflySbusDecoder|Hi91Imu|OnboardImu)\b", line):
                violations.append(f"{path.relative_to(root)}:{number}: algorithm uses motor device service")
            if directory == "src/chassis" and re.search(
                    r"\b(dtg1|fixed_motor_batch)\b", line):
                violations.append(f"{path.relative_to(root)}:{number}: chassis depends on transport batch format")
            if directory == "src/chassis" and "protocols::" in line:
                violations.append(f"{path.relative_to(root)}:{number}: protocol type")

# 自有代码统一使用 dust_control 根命名空间。
for directory in ("src", "tests"):
    for path in (root / directory).rglob("*"):
        if path.suffix not in (".cpp", ".cc", ".h", ".hpp"):
            continue
        relative = path.relative_to(root).as_posix()
        if path.suffix in (".h", ".hpp", ".hh", ".hxx"):
            if len(re.findall(r"^\s*#\s*pragma\s+once\s*$", path.read_text(), re.M)) != 1:
                violations.append(f"{relative}: header must use exactly one pragma once")
        for number, line in enumerate(path.read_text().splitlines(), 1):
            if re.search(r"\bdust_control::modules\b", line):
                violations.append(f"{relative}:{number}: chassis code must use dust_control::chassis")
            declaration = re.match(
                r"\s*namespace\s+([A-Za-z_]\w*(?:::\w+)*)\s*(?:\{|$)", line)
            if declaration and declaration[1].split("::")[0] != "dust_control":
                violations.append(f"{relative}:{number}: namespace must use dust_control root")
            if re.match(r"\s*namespace\s+(dust|wbr|modules|protocols|platform|msg)\s*=", line):
                violations.append(f"{relative}:{number}: obsolete root namespace alias")

links = (root / "src/chassis/CMakeLists.txt").read_text()
for block in re.findall(r"target_link_libraries\(dust_chassis\s+([^)]*)\)", links):
    for dependency in ("dust_ethercat", "dust_motors", "dust_remote", "dust_imu", "dust_chassis_runtime", "dust_startup", "yaml-cpp"):
        if dependency in block.split():
            violations.append(f"dust_chassis links {dependency}")

if violations:
    raise SystemExit("\n".join(violations))
print("Controller/device/transport dependency boundaries passed")

# EtherCAT branch must not retain USB source or link dependencies.
for path in (root / "src").rglob("*"):
    if path.suffix in (".cpp", ".hpp", ".h", ".cc", ".txt"):
        text = path.read_text()
        if "libusb" in text or "hardware/usb/" in text or "dust_usb" in text:
            raise SystemExit(f"USB dependency remains: {path}")
