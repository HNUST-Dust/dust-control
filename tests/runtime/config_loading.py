import copy
from pathlib import Path
import subprocess
import sys
import tempfile
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from startup_support import node_command, run_for

fixture, config_dir, controller = sys.argv[1:]
config_dir = Path(config_dir)
motors = yaml.safe_load((config_dir / "motors_params.yaml").read_text())
sensors = yaml.safe_load((config_dir / "sensors_params.yaml").read_text())
chassis = yaml.safe_load((config_dir / "chassis_params.yaml").read_text())


def motor(document, name):
    return next(device for device in document["motors"] if device["name"] == name)


def write(folder, m, s, c=None):
    folder.mkdir(exist_ok=True)
    deployment = yaml.safe_load((config_dir / "robot.yaml").read_text())
    deployment["execution"] = {
        "simulate": True,
        "realtime": False,
        "enable_output": False,
    }
    (folder / "robot.yaml").write_text(yaml.safe_dump(deployment))
    (folder / "motors_params.yaml").write_text(yaml.safe_dump(m))
    (folder / "sensors_params.yaml").write_text(yaml.safe_dump(s))
    (folder / "chassis_params.yaml").write_text(
        yaml.safe_dump(chassis if c is None else c)
    )


def run(*args, ok=True):
    result = subprocess.run(args, capture_output=True, text=True, timeout=6)
    assert (result.returncode == 0) == ok, (args, result.stdout, result.stderr)
    return result


with tempfile.TemporaryDirectory() as temp:
    root = Path(temp)
    valid = root / "valid"
    motor(motors, "left_wheel").update(command_slot=1, feedback_ids=[0x202], freshness_deadline_us=1000)
    motor(motors, "left_wheel")["command_sign"] = 1
    motor(motors, "right_wheel")["command_sign"] = -1
    sensors["hi91"]["freshness_deadline_us"] = 1000
    sensors["remote"]["freshness_deadline_us"] = 1501
    chassis["mechanics"]["joint_b_zero_offset_rad"] = 0.1
    chassis["mechanics"]["half_wheel_track_m"] = 0.3
    chassis["mechanics"]["wheel_radius_m"] = 0.07
    chassis["mechanics"]["leg_geometry_m"]["gh"] = 0.27
    chassis["balance"].update(
        theta_bias_deg=7.0,
        remote_deadband=0.1,
        max_body_speed_mps=0.8,
        max_turn_wheel_speed_mps=1.2,
    )
    chassis["gas_spring"].update(left_spring_force_n=0.0, right_spring_force_n=0.0)
    chassis["vmc"]["leg_length_kp"] = 900.0
    chassis["actuator"]["joint_torque_limit_nm"] = 3.0
    chassis["safety"]["tilt_cutoff_deg"] = 20.0
    # 总线和设备顺序不影响 name 对应的底盘反馈/目标槽位。
    motors["motors"].reverse()
    write(valid, motors, sensors)
    run(fixture, str(valid))
    for case in (
        "missing",
        "unknown",
        "zero",
        "overflow",
        "negative",
        "duplicate",
        "bus",
        "slot",
        "sensor",
        "version",
        "motor_version",
        "previous_motor_version",
        "sign",
        "legacy_current",
        "current_zero",
        "current_nan",
        "legacy_limit",
        "legacy_range",
        "wheel_command",
        "wheel_feedback",
        "duplicate_name",
        "unknown_name",
        "missing_device",
        "protocol_missing",
        "protocol_unknown",
        "protocol_mismatch",
        "feedback_empty",
        "feedback_multiple",
        "feedback_overflow",
        "buses_map",
        "devices_map",
        "old_schema_three",
        "owner_missing",
        "owner_empty",
        "owner_null",
        "owner_unknown",
        "chassis_missing",
        "chassis_unknown",
        "chassis_nan",
        "chassis_zero",
        "chassis_inverted",
        "chassis_version",
        "chassis_duplicate",
        "chassis_file",
    ):
        m, s, c = copy.deepcopy(motors), copy.deepcopy(sensors), copy.deepcopy(chassis)
        if case == "missing":
            del motor(m, "left_wheel")["command_id"]
        elif case == "unknown":
            motor(m, "left_wheel")["typo"] = 1
        elif case == "zero":
            motor(m, "left_wheel")["freshness_deadline_us"] = 0
        elif case == "overflow":
            motor(m, "left_wheel")["freshness_deadline_us"] = 2 ** 40
        elif case == "negative":
            s["remote"]["freshness_deadline_us"] = -1
        elif case == "duplicate":
            motor(m, "right_wheel").update(bus=3, command_slot=1, feedback_ids=[0x202])
        elif case == "bus":
            motor(m, "left_wheel")["bus"] = 1
        elif case == "slot":
            motor(m, "left_wheel")["command_slot"] = 4
        elif case == "sensor":
            s["extra"] = {}
        elif case == "version":
            s["schema_version"] = 99
        elif case == "motor_version":
            m["schema_version"] = 1
        elif case == "sign":
            motor(m, "left_wheel")["command_sign"] = 0
        elif case == "legacy_current":
            m["dji"] = {"current_per_nm": 3450.0}
        elif case == "current_zero":
            m["dji"] = {"current_per_nm": 0}
        elif case == "current_nan":
            m["dji"] = {"current_per_nm": float("nan")}
        elif case == "previous_motor_version":
            m["schema_version"] = 2
        elif case == "legacy_limit":
            m["dji"] = {"protocol_current_limit": 2000}
        elif case == "legacy_range":
            m["dm_range"] = {"torque_max_nm": 30}
        elif case == "wheel_command":
            motor(m, "left_wheel")["command_id"] = 0x201
        elif case == "wheel_feedback":
            motor(m, "left_wheel")["feedback_ids"] = [0x203]
        elif case == "duplicate_name":
            m["motors"].append(copy.deepcopy(motor(m, "left_wheel")))
        elif case == "unknown_name":
            motor(m, "left_wheel")["name"] = "unknown"
        elif case == "missing_device":
            m["motors"].pop()
        elif case == "protocol_missing":
            del motor(m, "left_joint_b")["protocol"]
        elif case == "protocol_unknown":
            motor(m, "left_joint_b")["protocol"] = "unknown"
        elif case == "protocol_mismatch":
            wheel = motor(m, "left_wheel")
            wheel["protocol"] = "dm"
            del wheel["command_slot"]
            del wheel["command_sign"]
        elif case == "feedback_empty":
            motor(m, "left_wheel")["feedback_ids"] = []
        elif case == "feedback_multiple":
            motor(m, "left_wheel")["feedback_ids"] = [0x202, 0x203]
        elif case == "feedback_overflow":
            motor(m, "left_joint_b")["feedback_ids"] = [0x800]
        elif case == "buses_map":
            m["motors"] = {}
        elif case == "devices_map":
            m["motors"][0] = {}
        elif case == "old_schema_three":
            m["schema_version"] = 3
        elif case == "owner_missing":
            del motor(m, "left_wheel")["adapter"]
        elif case == "owner_empty":
            motor(m, "left_wheel")["adapter"] = ""
        elif case == "owner_null":
            motor(m, "left_wheel")["adapter"] = None
        elif case == "owner_unknown":
            motor(m, "left_wheel")["adapter"] = "unknown"
        elif case == "chassis_missing":
            del c["mechanics"]["wheel_radius_m"]
        elif case == "chassis_unknown":
            c["balance"]["unknown"] = 1
        elif case == "chassis_nan":
            c["vmc"]["leg_length_kp"] = float("nan")
        elif case == "chassis_zero":
            c["mechanics"]["half_wheel_track_m"] = 0
        elif case == "chassis_inverted":
            c["balance"]["max_leg_length_m"] = 0.1
        elif case == "chassis_version":
            c["schema_version"] = 1
        bad = root / case
        write(bad, m, s, c)
        if case == "chassis_duplicate":
            with (bad / "chassis_params.yaml").open("a") as out:
                out.write("balance: {}\n")
        if case == "chassis_file":
            (bad / "chassis_params.yaml").unlink()
        if controller:
            result = run(*node_command(controller, bad), ok=False)
            assert "Chassis: 1 kHz" not in result.stdout
        run(fixture, str(valid), str(bad))
    if controller:
        result = run_for(node_command(controller, valid))
        assert result.returncode == 0, (result.stdout, result.stderr)
        missing = root / "absent"
        run(*node_command(controller, missing), ok=False)
print(
    "Module configuration loading, strict validation, atomic startup failure and instance isolation passed"
)
