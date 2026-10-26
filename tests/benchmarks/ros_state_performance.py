#!/usr/bin/env python3
"""Measure latest-state ROS and real Foxglove Bridge paths without hardware."""
import argparse
from contextlib import ExitStack
import asyncio
import json
import math
import os
from pathlib import Path
import platform
import signal
import socket
import subprocess
import tempfile
import threading
import time
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from startup_support import configuration

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.utilities import get_rmw_implementation_identifier
from dust_control.msg import ChassisTelemetry
from websockets.asyncio.client import connect


def summary(values):
    values = sorted(values)
    if not values:
        return None
    return {
        name: values[min(int((len(values) - 1) * fraction), len(values) - 1)]
        for name, fraction in (
            ("min", 0),
            ("p50", 0.5),
            ("p95", 0.95),
            ("p99", 0.99),
            ("max", 1),
        )
    }


def cpu_seconds(pid):
    fields = Path(f"/proc/{pid}/stat").read_text().split(") ", 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")


def rss_mib(pid):
    return (
        int(Path(f"/proc/{pid}/statm").read_text().split()[1])
        * os.sysconf("SC_PAGE_SIZE")
        / 1048576
    )


def process_tree(pid):
    result = [pid]
    try:
        children = Path(f"/proc/{pid}/task/{pid}/children").read_text().split()
        for child in children:
            result.extend(process_tree(int(child)))
    except FileNotFoundError:
        pass
    return result


def process_stats(process):
    pids = process_tree(process.pid)
    return sum(cpu_seconds(pid) for pid in pids), sum(rss_mib(pid) for pid in pids)


def stop(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)


class Measurements:
    def __init__(self):
        self.active = False
        self.rows = []
        self.payload_bytes = 0

    def record(self, message, size):
        if self.active:
            self.rows.append(
                (
                    time.monotonic_ns(),
                    message.monotonic_ns,
                    message.sequence,
                    message.missed_releases,
                    message.queue_drops,
                    message.control_duration_us,
                )
            )
            self.payload_bytes += size

    def report(self, duration):
        rows = self.rows
        if not rows:
            raise RuntimeError("No messages received during measurement")
        unique = len({row[2] for row in rows})
        span = rows[-1][2] - rows[0][2] + 1
        return {
            "received": len(rows),
            "receive_hz": len(rows) / duration,
            "unique_sequences": unique,
            "duplicates": len(rows) - unique,
            "unobserved_sequences": max(0, span - unique),
            "note": "Sequence gaps include publisher coalescing and transport loss; not a network packet-loss measurement.",
            "source_age_ms": summary([(row[0] - row[1]) / 1e6 for row in rows]),
            "arrival_interval_ms": summary(
                [(b[0] - a[0]) / 1e6 for a, b in zip(rows, rows[1:])]
            ),
            "payload_bytes_per_second": self.payload_bytes / duration,
            "source_missed_releases_delta": rows[-1][3] - rows[0][3],
            "source_queue_drops_delta": rows[-1][4] - rows[0][4],
            "control_duration_us": summary([row[5] for row in rows]),
        }


async def websocket_reader(port, measurements, ready):
    deadline = time.monotonic() + 10
    while True:
        try:
            ws = await connect(
                f"ws://127.0.0.1:{port}",
                proxy=None,
                subprotocols=["foxglove.sdk.v1", "foxglove.websocket.v1"],
            )
            break
        except OSError:
            if time.monotonic() > deadline:
                raise RuntimeError("Bridge WebSocket did not start")
            await asyncio.sleep(0.05)
    async with ws:
        async for frame in ws:
            if isinstance(frame, str):
                event = json.loads(frame)
                if event["op"] == "advertise":
                    for channel in event["channels"]:
                        if channel["topic"] == "/dust_control/chassis/telemetry":
                            assert channel["encoding"] == "cdr" and channel.get(
                                "schema"
                            )
                            await ws.send(
                                json.dumps(
                                    {
                                        "op": "subscribe",
                                        "subscriptions": [
                                            {"id": 1, "channelId": channel["id"]}
                                        ],
                                    }
                                )
                            )
            elif frame[0] == 1:
                message = deserialize_message(frame[13:], ChassisTelemetry)
                measurements.record(message, len(frame))
                ready.set()


async def scenario(args, environment, name, use_bridge, slow):
    ros_measurements, ws_measurements = Measurements(), Measurements()
    node = rclpy.create_node(f"state_perf_{name}")
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        durability=DurabilityPolicy.VOLATILE,
    )
    received = threading.Event()
    # frame_id='base_link' makes the real CDR sample larger than its empty default.
    sample = ChassisTelemetry()
    sample.header.frame_id = "base_link"
    message_size = len(serialize_message(sample))

    def callback(message):
        ros_measurements.record(message, message_size)
        received.set()
        if slow:
            time.sleep(0.05)

    subscription = node.create_subscription(
        ChassisTelemetry, "/dust_control/chassis/telemetry", callback, qos
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()
    controller = bridge = None
    ws_task = None
    config_context = ExitStack()
    config = config_context.enter_context(configuration(realtime=args.realtime))
    log = tempfile.TemporaryFile(mode="w+")
    try:
        command = [str(args.binary)]
        if args.source == "controller":
            command += [
                "--ros-args",
                "-p",
                "config_directory:=" + str(config),
            ]
        controller = subprocess.Popen(
            command, env=environment, stdout=log, stderr=log, start_new_session=True
        )
        deadline = time.monotonic() + 8
        while not received.is_set():
            if controller.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError("Controller did not produce ROS state")
            await asyncio.sleep(0.02)
        publisher_info = node.get_publishers_info_by_topic(
            "/dust_control/chassis/telemetry"
        )
        assert len(publisher_info) == 1
        publisher_qos = publisher_info[0].qos_profile
        assert publisher_qos.reliability == ReliabilityPolicy.BEST_EFFORT, publisher_qos
        # Some RMW discovery implementations report history depth as UNKNOWN (0).
        assert publisher_qos.depth in (0, 1), publisher_qos
        assert publisher_qos.durability == DurabilityPolicy.VOLATILE
        if use_bridge:
            with socket.socket() as sock:
                sock.bind(("127.0.0.1", 0))
                port = sock.getsockname()[1]
            bridge = subprocess.Popen(
                ["ros2", "launch", str(args.launch), f"port:={port}"],
                env=environment,
                stdout=log,
                stderr=log,
                start_new_session=True,
            )
            ready = asyncio.Event()
            ws_task = asyncio.create_task(
                websocket_reader(port, ws_measurements, ready)
            )
            await asyncio.wait_for(ready.wait(), timeout=12)
            deadline = time.monotonic() + 3
            while True:
                endpoints = node.get_subscriptions_info_by_topic(
                    "/dust_control/chassis/telemetry"
                )
                bridge_endpoints = [
                    entry for entry in endpoints if entry.node_name == "foxglove_bridge"
                ]
                if bridge_endpoints or time.monotonic() > deadline:
                    break
                await asyncio.sleep(0.02)
            assert len(bridge_endpoints) == 1, [
                (entry.node_name, str(entry.qos_profile)) for entry in endpoints
            ]
            bridge_qos = bridge_endpoints[0].qos_profile
            assert bridge_qos.reliability == ReliabilityPolicy.BEST_EFFORT, bridge_qos
            assert bridge_qos.depth in (0, 1), bridge_qos
        await asyncio.sleep(args.warmup)
        processes = {"publisher_process": controller}
        if bridge is not None:
            processes["bridge_tree"] = bridge
        before = {key: process_stats(value) for key, value in processes.items()}
        observer_cpu = time.process_time()
        start = time.monotonic()
        ros_measurements.active = ws_measurements.active = True
        while time.monotonic() - start < args.seconds:
            if controller.poll() is not None or (bridge and bridge.poll() is not None):
                raise RuntimeError("A measured process stopped")
            if ws_task is not None and ws_task.done():
                ws_task.result()
                raise RuntimeError("Bridge client stopped")
            await asyncio.sleep(0.1)
        ros_measurements.active = ws_measurements.active = False
        duration = time.monotonic() - start
        after = {key: process_stats(value) for key, value in processes.items()}
        report = {
            "name": name,
            "duration_s": duration,
            "cdr_bytes": message_size,
            "ros": ros_measurements.report(duration),
            "processes": {},
        }
        if use_bridge:
            report["foxglove"] = ws_measurements.report(duration)
        for key in processes:
            report["processes"][key] = {
                "cpu_percent_one_core": (after[key][0] - before[key][0])
                / duration
                * 100,
                "rss_mib_end": after[key][1],
            }
        report["collector_cpu_percent_one_core"] = (
            (time.process_time() - observer_cpu) / duration * 100
        )
        return report
    except BaseException:
        log.seek(0)
        print(log.read())
        raise
    finally:
        ros_measurements.active = ws_measurements.active = False
        if ws_task:
            ws_task.cancel()
            await asyncio.gather(ws_task, return_exceptions=True)
        stop(bridge)
        stop(controller)
        # Separate executor avoids letting a slow reader block the WebSocket client.
        executor.shutdown(timeout_sec=3)
        spin_thread.join(timeout=3)
        node.destroy_node()
        log.close()
        config_context.close()


async def main(args):
    # Local simulation only; results are not an RT or robot-hardware acceptance test.
    with tempfile.TemporaryDirectory(prefix="dust-state-perf-") as directory:
        prefix = Path(directory)
        marker = prefix / "share/ament_index/resource_index/packages/dust_control"
        marker.parent.mkdir(parents=True)
        marker.touch()
        resource = marker.parents[1] / "rosidl_interfaces/dust_control"
        resource.parent.mkdir()
        resource.write_text("msg/ChassisTelemetry.msg\n")
        share = prefix / "share/dust_control"
        share.mkdir()
        (share / "msg").symlink_to(
            args.launch.parents[1] / "msg", target_is_directory=True
        )
        (prefix / "lib").symlink_to(args.binary.parent, target_is_directory=True)
        environment = dict(os.environ)
        environment[
            "AMENT_PREFIX_PATH"
        ] = f"{prefix}:{args.bridge_prefix}:" + environment.get("AMENT_PREFIX_PATH", "")
        environment[
            "LD_LIBRARY_PATH"
        ] = f"{args.binary.parent}:{args.bridge_prefix}/lib:" + environment.get(
            "LD_LIBRARY_PATH", ""
        )
        report = {
            "environment": {
                "platform": platform.platform(),
                "rmw": get_rmw_implementation_identifier(),
                "cpu_affinity": sorted(os.sched_getaffinity(0)),
                "realtime": args.realtime,
                "source": args.source + ", same-host ROS and WebSocket clients",
                "synthetic_source_busy_wait": args.source == "synthetic",
                "warmup_seconds": args.warmup,
            },
            "scenarios": [],
        }
        for name, bridge, slow in (
            ("ros_fast", False, False),
            ("ros_and_foxglove", True, False),
            ("slow_ros_and_foxglove", True, True),
        ):
            result = await scenario(args, environment, name, bridge, slow)
            report["scenarios"].append(result)
            args.output.write_text(json.dumps(report, indent=2) + "\n")
            print(name, json.dumps(result), flush=True)
            await asyncio.sleep(0.5)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--realtime",
        action="store_true",
        help="Controller only: use its existing RT scheduling (requires privileges and unlimited memlock)",
    )
    parser.add_argument(
        "--source", choices=("controller", "synthetic"), default="controller"
    )
    parser.add_argument("--binary", type=Path)
    parser.add_argument("--bridge-prefix", type=Path, required=True)
    parser.add_argument(
        "--launch", type=Path, default=Path("launch/foxglove.launch.py")
    )
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--warmup", type=float, default=2)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if (
        not math.isfinite(args.seconds)
        or not math.isfinite(args.warmup)
        or args.seconds <= 0
        or args.warmup < 0
    ):
        parser.error(
            "seconds must be finite and positive; warmup must be finite and nonnegative"
        )
    if args.realtime and args.source != "controller":
        parser.error("--realtime only applies to the controller source")
    if args.binary is None:
        args.binary = Path(
            "build/dust_control_node"
            if args.source == "controller"
            else "build/ros_state_benchmark_publisher"
        )
    if (
        args.source == "synthetic"
        and args.binary.name != "ros_state_benchmark_publisher"
    ):
        parser.error(
            "synthetic source requires the test-only ros_state_benchmark_publisher"
        )
    args.binary = args.binary.resolve()
    args.bridge_prefix = args.bridge_prefix.resolve()
    args.launch = args.launch.resolve()
    rclpy.init()
    try:
        asyncio.run(main(args))
    finally:
        rclpy.shutdown()
