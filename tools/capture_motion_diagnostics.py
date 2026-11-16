#!/usr/bin/env python3
"""Record every delivered control sample for wheel-reliability calibration (read only)."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import queue
import subprocess
import threading
import time

from capture_balance_diagnostics import json_native


MINIMUM_VERSION = 9


def finite_json(value):
    """Use standard JSON null for missing measurements, never turn them into zeros."""
    value = json_native(value)
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: finite_json(item) for key, item in value.items()}
    if isinstance(value, list):
        return [finite_json(item) for item in value]
    return value


def sample_record(message, scenario, condition, received_ns, recorder_drops):
    record = {}
    for name in message.get_fields_and_field_types():
        if name == 'header':
            record['stamp_ns'] = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        else:
            record[name] = json_native(getattr(message, name))
    record.update(scenario=scenario, condition=condition,
                  recorder_received_monotonic_ns=received_ns,
                  recorder_queue_drops=recorder_drops)
    return finite_json(record)


def configuration_metadata(directory):
    """Supplied files are provenance, not proof of what the running node loaded."""
    files = {}
    for name in ('dust_control.yaml', 'chassis_params.yaml',
                 'sensors_params.yaml', 'motors_params.yaml'):
        path = directory / name
        content = path.read_bytes()
        files[name] = {'sha256': hashlib.sha256(content).hexdigest(),
                       'text': content.decode()}
    def git(*arguments):
        result = subprocess.run(['git', '-C', str(directory), *arguments],
                                capture_output=True, check=False)
        return result.stdout if result.returncode == 0 else None
    revision = git('rev-parse', 'HEAD')
    tracked_diff = git('diff', 'HEAD', '--binary')
    return {'config_directory': str(directory.resolve()), 'files': files,
            'git_revision': revision.decode().strip() if revision else None,
            'tracked_diff_sha256': hashlib.sha256(tracked_diff).hexdigest()
                if tracked_diff is not None else None,
            'git_status': (git('status', '--porcelain') or b'').decode(),
            'running_configuration_verified': False,
            'note': 'Supplied files may differ from the running process; actual KF/threshold parameters are recorded in valid samples.'}


class JsonlWriter:
    """Bounded non-RT disk queue: never let a slow disk silently block reception."""
    def __init__(self, path, capacity=8192):
        self.stream = path.open('x')
        self.queue = queue.Queue(maxsize=capacity)
        self.drops = 0
        self.written = 0
        self.error = None
        self.thread = threading.Thread(target=self._run, name='motion-log-writer')
        self.thread.start()

    def submit(self, record):
        if self.error:
            return
        try:
            self.queue.put_nowait(record)
        except queue.Full:
            self.drops += 1

    def _run(self):
        last_flush = time.monotonic()
        try:
            while True:
                record = self.queue.get()
                if record is None:
                    break
                self.stream.write(json.dumps(record, separators=(',', ':'), allow_nan=False) + '\n')
                self.written += 1
                if time.monotonic() - last_flush >= 1.:
                    self.stream.flush()
                    last_flush = time.monotonic()
        except Exception as error:
            self.error = f'{type(error).__name__}: {error}'
        finally:
            try:
                self.stream.close()
            except Exception as error:
                self.error = self.error or f'{type(error).__name__}: {error}'

    def close(self):
        # Drain queued samples; terminate cleanly even if the writer already failed.
        while self.thread.is_alive():
            try:
                self.queue.put(None, timeout=.1)
                break
            except queue.Full:
                continue
        self.thread.join()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config-directory', type=Path, required=True)
    parser.add_argument('--scenario', required=True, help='e.g. flat_reverse, spin_ccw, small_projectile')
    parser.add_argument('--condition', choices=('normal', 'anomaly', 'unlabeled'), required=True,
                        help='External experiment label, NOT inferred from filter status')
    parser.add_argument('--notes', default='')
    parser.add_argument('--topic', default='/dust_control/chassis/telemetry_samples')
    parser.add_argument('--duration', type=float, default=30.)
    parser.add_argument('--wait', type=float, default=120.)
    args = parser.parse_args()
    if any(not math.isfinite(value) or value <= 0. for value in (args.duration, args.wait)):
        parser.error('duration and wait must be finite and positive')
    paths = [args.output, args.output.with_suffix('.metadata.json'), args.output.with_suffix('.summary.json')]
    if any(path.exists() for path in paths):
        parser.error('output or sidecar already exists; choose a new name')
    try:
        metadata = configuration_metadata(args.config_directory)
    except OSError as error:
        parser.error(f'Cannot snapshot configuration: {error}')
    metadata.update(format_version=1, minimum_telemetry_version=MINIMUM_VERSION,
                    topic=args.topic, scenario=args.scenario, condition=args.condition,
                    notes=args.notes, started_unix_ns=time.time_ns(),
                    requested_duration_s=args.duration,
                    receipt_clock_note='Recorder and source monotonic clocks may belong to different hosts.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    paths[1].write_text(json.dumps(metadata, indent=2, allow_nan=False) + '\n')

    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.executors import ExternalShutdownException
    from dust_control.msg import ChassisTelemetry

    rclpy.init()
    node = rclpy.create_node('motion_diagnostic_recorder')
    writer = JsonlWriter(args.output)
    start = None
    received = 0
    invalid_estimator = 0
    simulated = 0
    failure = None
    versions = set()
    armed = time.monotonic()

    def receive(message):
        nonlocal start, received, invalid_estimator, simulated
        if message.version < MINIMUM_VERSION:
            raise RuntimeError(f'Telemetry version {message.version}; rebuild both endpoints for version {MINIMUM_VERSION}')
        if start is None:
            start = time.monotonic()
            print('CAPTURING: all delivered samples, including invalid/rejected/disabled states.', flush=True)
        received += 1
        invalid_estimator += not message.motion_estimator_valid
        simulated += message.simulated
        versions.add(message.version)
        writer.submit(sample_record(message, args.scenario, args.condition,
                                    time.monotonic_ns(), writer.drops))

    qos = QoSProfile(depth=1024, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    subscription = node.create_subscription(ChassisTelemetry, args.topic, receive, qos)
    print(f'WAITING: {args.topic}; no controller or zero-command arming filter.', flush=True)
    try:
        while rclpy.ok():
            now = time.monotonic()
            if start is not None and now - start >= args.duration:
                break
            if start is None and now - armed >= args.wait:
                raise RuntimeError('No diagnostic samples received before timeout')
            if writer.error:
                raise RuntimeError(writer.error)
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        failure = f'{type(error).__name__}: {error}'
    finally:
        writer.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    failure = failure or writer.error
    summary = {'received_samples': received, 'written_samples': writer.written,
               'recorder_queue_drops': writer.drops, 'invalid_estimator_samples': invalid_estimator,
               'simulated_samples': simulated, 'telemetry_versions': sorted(versions),
               'elapsed_capture_s': None if start is None else time.monotonic() - start,
               'error': failure, 'output': str(args.output)}
    paths[2].write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    print(json.dumps(summary), flush=True)
    if failure or not writer.written:
        raise SystemExit(failure or 'No samples written')


if __name__ == '__main__':
    main()
