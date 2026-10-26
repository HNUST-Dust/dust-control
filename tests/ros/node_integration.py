"""ROS observation-only graph, YAML execution, immutable path and clean shutdown."""
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from dust_control.msg import ChassisTelemetry
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from startup_support import configuration, run_for

binary = sys.argv[1]
namespace = f"/dust_test_{os.getpid()}"
rclpy.init()
observer = rclpy.create_node("dust_test_observer")
received, telemetry = [], []
telemetry_subscription = observer.create_subscription(
    ChassisTelemetry,
    namespace + "/dust_control/chassis/telemetry",
    telemetry.append,
    QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    ),
)
subscription = observer.create_subscription(
    DiagnosticArray, namespace + "/dust_control/status", received.append, 1
)

with configuration() as config:
    command = [binary, "--ros-args", "-p", "config_directory:=" + str(config)]
    process = subprocess.Popen(
        command + ["-r", "__ns:=" + namespace],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        deadline = time.monotonic() + 8
        while (
            (not received or not telemetry)
            and time.monotonic() < deadline
            and process.poll() is None
        ):
            rclpy.spin_once(observer, timeout_sec=0.05)
        assert received and telemetry, (process.poll(), "No ROS observations received")
        observation = telemetry[-1]
        assert observation.version == 1 and observation.sequence > 0
        assert observation.simulated and observation.source_connected
        assert not observation.imu_fresh and math.isnan(observation.pitch_deg)
        assert not observation.left_leg_fresh and math.isnan(observation.left_length_m)
        status = received[-1].status[0]
        values = {entry.key: entry.value for entry in status.values}
        assert (
            values["output_requested"] == "false" and status.hardware_id == "simulation"
        )
        # 话题数据可能先于 ROS 图节点发现到达；等待被查询节点出现再检查订阅。
        discovery_deadline = time.monotonic() + 3
        while ("dust_control", namespace) not in observer.get_node_names_and_namespaces():
            assert process.poll() is None, (process.returncode, "ROS node exited during discovery")
            assert time.monotonic() < discovery_deadline, "ROS node discovery timed out"
            rclpy.spin_once(observer, timeout_sec=0.05)
        subscriptions = observer.get_subscriber_names_and_types_by_node(
            "dust_control", namespace
        )
        assert not any(
            name.endswith(("/cmd_vel", "/motion_command")) for name, _ in subscriptions
        )
        client = observer.create_client(
            SetParameters, namespace + "/dust_control/set_parameters"
        )
        assert client.wait_for_service(timeout_sec=3)
        request = SetParameters.Request()
        request.parameters = [
            Parameter("config_directory", value="/other").to_parameter_msg()
        ]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(observer, future, timeout_sec=3)
        assert future.done() and not future.result().results[0].successful
        process.send_signal(signal.SIGINT)
        stdout, stderr = process.communicate(timeout=5)
        assert process.returncode == 0, (stdout, stderr)
        assert "Stopped;" in stdout and "output=disabled" in stdout
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()
        observer.destroy_node()
        rclpy.shutdown()

    for param in (
        "simulate:=true",
        "enable_output:=true",
        "realtime:=false",
        "run_seconds:=1",
        "command_source:=ros",
        "command_timeout_ms:=200",
        "usb_serial:=obsolete",
    ):
        result = subprocess.run(
            command + ["-p", param], capture_output=True, text=True, timeout=5
        )
        assert result.returncode == 1 and "dust_control.yaml" in result.stderr, result

    document = yaml.safe_load((config / "dust_control.yaml").read_text())
    document["execution"]["enable_output"] = True
    (config / "dust_control.yaml").write_text(yaml.safe_dump(document))
    result = subprocess.run(command, capture_output=True, text=True, timeout=5)
    assert result.returncode == 1 and "Simulation cannot enable" in result.stderr
    document["execution"]["enable_output"] = False
    document["modules"]["chassis"]["enabled"] = False
    document["modules"]["gimbal"]["enabled"] = False
    document["modules"]["ui"]["adapter"] = None
    (config / "dust_control.yaml").write_text(yaml.safe_dump(document))
    result = run_for(command)
    assert result.returncode == 0, (result.stdout, result.stderr)
    assert "Chassis: 1 kHz" not in result.stdout

if len(sys.argv) > 2:
    for disconnect in (False, True):
        environment = dict(
            os.environ, LD_PRELOAD=sys.argv[2], WBR_MOCK_REQUIRE_PREPOST="1"
        )
        if disconnect:
            environment["WBR_MOCK_DISCONNECT"] = "1"
        with configuration(simulate=False) as config:
            result = run_for(
                [binary, "--ros-args", "-p", "config_directory:=" + str(config)],
                env=environment,
                seconds=0.5,
            )
        assert result.returncode == int(disconnect), (
            result.returncode,
            result.stdout,
            result.stderr,
        )
        assert "Bridge capabilities=" in result.stdout
        if not disconnect:
            assert "output=disabled" in result.stdout and "Stopped;" in result.stdout
print("ROS observation-only graph, YAML startup, shutdown and USB faults passed")
