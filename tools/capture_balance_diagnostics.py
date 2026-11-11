#!/usr/bin/env python3
"""Read-only ROS telemetry capture, armed by active LQR14 and zero translation command."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from dust_control.msg import ChassisTelemetry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=20.)
    parser.add_argument('--wait', type=float, default=120.)
    args = parser.parse_args()
    if args.duration <= 0 or args.wait <= 0:
        parser.error('durations must be positive')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    stream = args.output.open('x')
    rclpy.init()
    node = rclpy.create_node('balance_diagnostic_recorder')
    start = None
    samples = 0
    eligible = 0
    last = None
    armed_time = time.monotonic()

    def receive(msg):
        nonlocal start, samples, eligible, last
        good = (msg.feedback_valid and msg.motion_diagnostics_valid and
                msg.lqr14_status == 2 and msg.lqr14_blend >= .999 and
                math.isfinite(msg.target_body_speed_mps) and
                abs(msg.target_body_speed_mps) < .001 and
                abs(msg.remote_chassis_x) < .05)
        if start is None:
            if not good:
                return
            start = time.monotonic()
            print(f'CAPTURING: {args.duration:g}-second window started; keep translation stick centered.', flush=True)
        record = {}
        for name in msg.get_fields_and_field_types():
            if name == 'header':
                record['stamp_ns'] = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
                continue
            value = getattr(msg, name)
            if not isinstance(value, (str, bool, int, float)):
                value = list(value)
            record[name] = value
        record['capture_eligible'] = good
        stream.write(json.dumps(record, separators=(',', ':')) + '\n')
        samples += 1
        eligible += int(good)
        last = msg.state

    subscription = node.create_subscription(ChassisTelemetry, '/dust_control/chassis/telemetry', receive,
                                             qos_profile_sensor_data)
    print('ARMED: waiting for active LQR14, blend=1, centered translation stick.', flush=True)
    try:
        while rclpy.ok():
            now = time.monotonic()
            if start is not None and now - start >= args.duration:
                break
            if start is None and now - armed_time >= args.wait:
                break
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        stream.close()
        node.destroy_node()
        rclpy.shutdown()
    summary = {'samples': samples, 'eligible_samples': eligible, 'last_state': last,
               'duration_s': None if start is None else time.monotonic()-start,
               'output': str(args.output), 'wheel_actual_torque_available': False}
    args.output.with_suffix('.summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary), flush=True)
    if not samples:
        raise SystemExit('No eligible data: no diagnosis possible.')


if __name__ == '__main__':
    main()
