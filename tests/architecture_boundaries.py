"""Keep the controller, device drivers and transport independently usable."""
from pathlib import Path
import re
import sys

root = Path(sys.argv[1])
violations = []
rules = {
    "src/chassis": ("hardware/", "runtime/", "telemetry/"),
    "src/hardware/usb": (),
    **{f"src/hardware/{group}": ("hardware/usb/", "chassis/", "runtime/", "telemetry/")
       for group in ("motors", "remote", "imu", "referee")},
}
# 底盘只引用设备侧的被动数据定义，不能引用 Decoder、Encoder 或通信实现。
chassis_data_headers = {
    "hardware/imu/imu_sample.hpp",
    "hardware/motors/motor_state.hpp",
    "hardware/motors/motor_targets.hpp",
    "hardware/motors/dji_model.hpp",
    "hardware/remote/remote_input_state.hpp",
}
for directory, forbidden in rules.items():
    for path in (root / directory).rglob("*"):
        if path.suffix not in (".cpp", ".cc", ".h", ".hpp"):
            continue
        for number, line in enumerate(path.read_text().splitlines(), 1):
            match = re.match(r'\s*#\s*include\s*[<"]([^>"]+)', line)
            is_chassis_runtime = directory == "src/chassis" and path.name in ("chassis_runtime.cpp", "chassis_runtime.hpp", "chassis_config_loading.cpp")
            if is_chassis_runtime and path.name != "chassis_config_loading.cpp" and match and match[1] in (
                "robot.hpp", "options.hpp", "hardware/usb/usb_session.hpp", "hardware/usb/usb_link.hpp",
                "hardware/config_yaml.hpp",
            ):
                violations.append(f"{path.relative_to(root)}:{number}: chassis runtime owns startup dependency")
            if (directory == "src/chassis" and not is_chassis_runtime and match
                    and match[1].endswith("chassis_runtime.hpp")):
                violations.append(f"{path.relative_to(root)}:{number}: algorithm includes business runtime")
            if (not is_chassis_runtime and match and match[1].startswith(forbidden)
                    and not (directory == "src/chassis" and match[1] in chassis_data_headers)):
                violations.append(f"{path.relative_to(root)}:{number}: {match[1]}")
            if path.relative_to(root).as_posix().removeprefix("src/") in chassis_data_headers:
                if ((match and match[1].startswith(("hardware/", "chassis/", "runtime/", "telemetry/")))
                        or "protocols::" in line):
                    violations.append(f"{path.relative_to(root)}:{number}: data header depends on implementation")
            if directory == "src/hardware/usb" and match and match[1].endswith("_protocol.h"):
                violations.append(f"{path.relative_to(root)}:{number}: USB includes device wire protocol")
            if directory == "src/chassis" and "protocols::" in line:
                violations.append(f"{path.relative_to(root)}:{number}: protocol type")

links = (root / "src/chassis/CMakeLists.txt").read_text()
for block in re.findall(r"target_link_libraries\(dust_chassis\s+([^)]*)\)", links):
    for dependency in ("dust_usb", "dust_motors", "dust_remote", "dust_imu", "dust_referee", "dust_telemetry", "dust_chassis_runtime", "dust_startup", "yaml-cpp"):
        if dependency in block.split():
            violations.append(f"dust_chassis links {dependency}")

if violations:
    raise SystemExit("\n".join(violations))
print("Controller/device/transport dependency boundaries passed")
