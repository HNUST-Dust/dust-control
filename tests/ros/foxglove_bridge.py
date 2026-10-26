"""One real WebSocket serves independent ROS nodes; Bridge can restart alone."""
import asyncio
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from dust_control.msg import ChassisTelemetry
from websockets.asyncio.client import connect

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from startup_support import configuration

binary, bridge_prefix, launch_file = sys.argv[1:]
build = Path(binary).parent
# 将 Bridge/DDS 测试负载移出控制核；控制线程仍自行绑定 CPU 3。
os.sched_setaffinity(0, os.sched_getaffinity(0) - {3})
rclpy.init()
observer = rclpy.create_node("independent_waveform_source")
publisher = observer.create_publisher(String, "/independent/waveform", 1)


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)


async def receive_waveforms(port, controller):
    deadline = time.monotonic() + 12
    while True:
        assert controller.poll() is None, "Control node stopped with Bridge"
        try:
            websocket = await connect(
                f"ws://127.0.0.1:{port}",
                subprotocols=["foxglove.sdk.v1", "foxglove.websocket.v1"],
                proxy=None,
            )
            break
        except OSError:
            assert time.monotonic() < deadline, "Bridge did not open its port"
            await asyncio.sleep(0.05)
    async with websocket:
        subscribed = {}
        decoded = set()
        sequence = 0
        while time.monotonic() < deadline and len(decoded) < 2:
            publisher.publish(String(data="independent-node"))
            rclpy.spin_once(observer, timeout_sec=0)
            try:
                frame = await asyncio.wait_for(websocket.recv(), timeout=0.1)
            except asyncio.TimeoutError:
                continue
            if isinstance(frame, str):
                event = json.loads(frame)
                if event["op"] == "serverInfo":
                    assert "clientPublish" not in event["capabilities"]
                    assert "services" not in event["capabilities"]
                    assert "parameters" not in event["capabilities"]
                if event["op"] != "advertise":
                    continue
                for channel in event["channels"]:
                    if (
                        channel["topic"]
                        in ("/dust_control/chassis/telemetry", "/independent/waveform")
                        and channel["topic"] not in subscribed.values()
                    ):
                        sub_id = len(subscribed) + 1
                        subscribed[sub_id] = channel["topic"]
                        assert channel["encoding"] == "cdr", channel
                        assert channel.get("schema"), channel
                        await websocket.send(
                            json.dumps(
                                {
                                    "op": "subscribe",
                                    "subscriptions": [
                                        {"id": sub_id, "channelId": channel["id"]}
                                    ],
                                }
                            )
                        )
            elif frame[0] == 1:
                topic = subscribed[struct.unpack_from("<I", frame, 1)[0]]
                msg_type = ChassisTelemetry if topic.endswith("/telemetry") else String
                message = deserialize_message(frame[13:], msg_type)
                if msg_type is ChassisTelemetry:
                    assert message.sequence > 0 and message.simulated
                    assert message.source_connected
                    sequence = message.sequence
                else:
                    assert message.data == "independent-node"
                decoded.add(topic)
        assert len(decoded) == 2, (subscribed, decoded)
        return sequence


with configuration() as config, tempfile.TemporaryDirectory(
    prefix="dust-bridge-test-"
) as directory:
    # Bridge uses the ament index to locate generated typesupport; no install required.
    prefix = Path(directory)
    marker = prefix / "share/ament_index/resource_index/packages/dust_control"
    marker.parent.mkdir(parents=True)
    marker.touch()
    interface_index = marker.parents[1] / "rosidl_interfaces" / "dust_control"
    interface_index.parent.mkdir(parents=True)
    interface_index.write_text("msg/ChassisTelemetry.msg\n")
    package_share = prefix / "share/dust_control"
    package_share.mkdir()
    (package_share / "msg").symlink_to(
        Path(launch_file).parents[1] / "msg", target_is_directory=True
    )
    (prefix / "lib").symlink_to(build, target_is_directory=True)
    environment = dict(os.environ)
    environment["AMENT_PREFIX_PATH"] = f"{prefix}:{bridge_prefix}:" + environment.get(
        "AMENT_PREFIX_PATH", ""
    )
    environment["LD_LIBRARY_PATH"] = f"{build}:{bridge_prefix}/lib:" + environment.get(
        "LD_LIBRARY_PATH", ""
    )
    with socket.socket() as socket_:
        socket_.bind(("127.0.0.1", 0))
        port = socket_.getsockname()[1]
    with tempfile.TemporaryFile(mode="w+") as log:
        controller = subprocess.Popen(
            [binary, "--ros-args", "-p", "config_directory:=" + str(config)],
            env=environment,
            stdout=log,
            stderr=log,
            start_new_session=True,
        )
        bridge = None
        try:
            last_sequence = 0
            for _ in range(2):
                bridge = subprocess.Popen(
                    ["ros2", "launch", launch_file, f"port:={port}"],
                    env=environment,
                    stdout=log,
                    stderr=log,
                    start_new_session=True,
                )
                sequence = asyncio.run(receive_waveforms(port, controller))
                assert sequence > last_sequence
                last_sequence = sequence
                stop(bridge)
                assert controller.poll() is None
            stop(controller)
            assert controller.returncode == 0
        except BaseException:
            log.seek(0)
            print(log.read(), file=sys.stderr)
            raise
        finally:
            if bridge is not None:
                stop(bridge)
            stop(controller)
observer.destroy_node()
rclpy.shutdown()
print("Shared Bridge decoded two ROS nodes; independent Bridge restart passed")
