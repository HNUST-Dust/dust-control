"""ROS graph, command validation, startup-only parameters and clean shutdown; no USB."""
import os
import signal
import subprocess
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from dust_control.msg import MotionCommand

binary = sys.argv[1]
namespace = f"/dust_test_{os.getpid()}"
rclpy.init()
observer = rclpy.create_node("dust_test_observer")
received = []
subscription = observer.create_subscription(
    DiagnosticArray, namespace + "/dust_control/status", received.append, 1)
velocity = observer.create_publisher(Twist, namespace + "/dust_control/cmd_vel", 1)
motion = observer.create_publisher(MotionCommand, namespace + "/dust_control/motion_command", 1)
process = subprocess.Popen([
    binary, "--ros-args", "-r", "__ns:=" + namespace,
    "-p", "simulate:=true", "-p", "realtime:=false",
    "-p", "command_source:=ros", "-p", "run_seconds:=0.0",
], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
try:
    deadline = time.monotonic() + 8
    while not received and time.monotonic() < deadline and process.poll() is None:
        rclpy.spin_once(observer, timeout_sec=0.05)
    assert received, (process.poll(), "No ROS status received")
    status = received[-1].status[0]
    values = {entry.key: entry.value for entry in status.values}
    assert values["output_requested"] == "false"
    assert values["command_source"] == "ros"
    assert status.hardware_id == "simulation"
    # Reliable, depth-one command endpoints are discoverable and reject bad axes/frames.
    deadline = time.monotonic() + 3
    while (velocity.get_subscription_count() == 0 or motion.get_subscription_count() == 0) and time.monotonic() < deadline:
        rclpy.spin_once(observer, timeout_sec=0.05)
    assert velocity.get_subscription_count() == 1 and motion.get_subscription_count() == 1
    invalid = Twist()
    invalid.linear.y = 1.0
    velocity.publish(invalid)
    invalid_motion = MotionCommand()
    invalid_motion.header.frame_id = "map"
    motion.publish(invalid_motion)
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        rclpy.spin_once(observer, timeout_sec=0.05)
        values = {entry.key: entry.value for entry in received[-1].status[0].values}
        if int(values["rejected_commands"]) >= 2:
            break
    assert int(values["rejected_commands"]) >= 2
    client = observer.create_client(SetParameters, namespace + "/dust_control/set_parameters")
    assert client.wait_for_service(timeout_sec=3)
    request = SetParameters.Request()
    request.parameters = [Parameter("enable_output", value=True).to_parameter_msg()]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(observer, future, timeout_sec=3)
    assert future.done() and not future.result().results[0].successful
    process.send_signal(signal.SIGINT)
    stdout, stderr = process.communicate(timeout=5)
    assert process.returncode == 0, (stdout, stderr)
    assert "Stopped;" in stdout and "output=disabled" in stdout
finally:
    if process.poll() is None:
        process.terminate()
        process.communicate(timeout=5)
    observer.destroy_node()
    rclpy.shutdown()

for params, message in (
    (["simulate:=true", "enable_output:=true"], "Simulation cannot enable"),
    (["command_source:=invalid"], "command_source"),
    (["command_timeout_ms:=0"], "command_timeout_ms"),
):
    args = [binary, "--ros-args", "-p", "realtime:=false"]
    for param in params:
        args += ["-p", param]
    result = subprocess.run(args, capture_output=True, text=True, timeout=5)
    assert result.returncode == 1 and message in result.stderr, result
print("ROS graph, custom commands, read-only parameters, shutdown and invalid startup passed")

if len(sys.argv) > 2:
    backend = sys.argv[2]
    for disconnect in (False, True):
        environment = dict(os.environ, LD_PRELOAD=backend, WBR_MOCK_REQUIRE_PREPOST="1")
        if disconnect:
            environment["WBR_MOCK_DISCONNECT"] = "1"
        result = subprocess.run([
            binary, "--ros-args", "-p", "realtime:=false", "-p", "run_seconds:=0.3",
        ], env=environment, capture_output=True, text=True, timeout=5)
        assert result.returncode == int(disconnect), (result.stdout, result.stderr)
        assert "Bridge capabilities=" in result.stdout
        if not disconnect:
            assert "output=disabled" in result.stdout and "Stopped;" in result.stdout
    print("ROS node USB mock handshake, control and disconnect propagation passed")
