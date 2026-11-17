#!/usr/bin/env python3
"""Summarize labeled motion logs and threshold exceedance; never change controller tuning."""
import argparse
from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path


RELIABILITY_PARAMETER_NAMES = (
    'contact_off_fraction', 'contact_on_fraction', 'contact_loss_delay_s',
    'recovery_time_s', 'pair_residual_limit_mps', 'innovation_floor_mps',
    'innovation_sigma', 'rejection_ratio', 'suspect_confidence')


def finite(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def percentile(values, percent):
    ordered = sorted(values)
    if not ordered:
        return None
    index = (len(ordered)-1) * percent / 100.
    low = int(index)
    high = min(low+1, len(ordered)-1)
    return ordered[low] + (ordered[high]-ordered[low]) * (index-low)


def distribution(values):
    values = [value for value in values if finite(value)]
    if not values:
        return {'samples': 0}
    mean = sum(values)/len(values)
    return {'samples': len(values), 'mean': mean,
            'std': math.sqrt(sum((value-mean)**2 for value in values)/len(values)),
            'rms': math.sqrt(sum(value*value for value in values)/len(values)),
            'min': min(values), 'max': max(values),
            **{f'p{p:g}': percentile(values, p) for p in (50, 90, 95, 99, 99.9)}}


def collect_signals(rows):
    """Use validity flags, NEVER select observations by trusted/rejected status."""
    signals = defaultdict(list)
    for row in rows:
        if row.get('imu_fresh'):
            for axis, value in enumerate(row.get('imu_raw_accel_g', [])):
                signals[f'imu_axis_{axis}_g'].append(value)
        if row.get('forward_acceleration_valid'):
            signals['forward_acceleration_mps2'].append(row.get('forward_acceleration_mps2'))
        # These remain useful for static/disabled and flight logs where the
        # balance estimator is not executed. NaN/null is excluded by distribution.
        for field in ('left_support_force_n', 'right_support_force_n',
                      'pitch_deg', 'roll_deg', 'yaw_rate_rad_s'):
            signals[field].append(row.get(field))
        if not row.get('motion_estimator_valid'):
            continue
        signals['prior_speed_variance'].append(row.get('motion_prior_speed_variance'))
        for side, name in enumerate(('left', 'right')):
            if row['motion_wheel_valid'][side]:
                residual = row['wheel_speed_prediction_residual_mps'][side]
                if finite(residual):
                    signals[f'{name}_prediction_abs_mps'].append(abs(residual))
            if row['wheel_pair_comparison_valid'][side]:
                residual = row['wheel_speed_pair_residual_mps'][side]
                if finite(residual):
                    signals[f'{name}_pair_abs_mps'].append(abs(residual))
            if row['wheel_prediction_gate_active'][side] and row['motion_wheel_valid'][side]:
                signals[f'{name}_prediction_ratio'].append(row['wheel_prediction_ratio'][side])
            if row['motion_support_valid']:
                signals[f'{name}_support_fraction'].append(row['motion_support_fraction'][side])
            if row['wheel_contact_known'][side]:
                signals[f'{name}_low_support_time_s'].append(row['wheel_low_support_time_s'][side])
    return {name: distribution(values) for name, values in signals.items()}


def threshold_scan(rows, pair_thresholds, innovation_floors):
    results = {'pair': [], 'innovation': []}
    for threshold in pair_thresholds:
        values = [abs(row['wheel_speed_pair_residual_mps'][0]) for row in rows
                  if row.get('motion_estimator_valid') and row['wheel_pair_comparison_valid'][0]
                  and finite(row['wheel_speed_pair_residual_mps'][0])]
        results['pair'].append({'threshold_mps': threshold, 'eligible_samples': len(values),
                                'exceedance_fraction': sum(v > threshold for v in values)/len(values) if values else None})
    for floor in innovation_floors:
        eligible = exceeded = 0
        for row in rows:
            variance = row.get('motion_prior_speed_variance')
            parameters = row.get('motion_reliability_parameters', [])
            if not row.get('motion_estimator_valid') or not finite(variance) or variance < 0 or len(parameters) != 9:
                continue
            sigma = parameters[6]
            if not finite(sigma):
                continue
            gate = max(floor, sigma*math.sqrt(variance))
            for side in range(2):
                residual = row['wheel_speed_prediction_residual_mps'][side]
                if row['wheel_prediction_gate_active'][side] and row['motion_wheel_valid'][side] and finite(residual):
                    eligible += 1
                    exceeded += abs(residual) > gate
        results['innovation'].append({'floor_mps': floor, 'eligible_side_samples': eligible,
                                      'exceedance_fraction': exceeded/eligible if eligible else None})
    return results


def contact_loss_runs(rows, side):
    """Observed continuous low-support intervals, split at source gaps/unknown force."""
    runs = []
    duration = 0.
    previous = None
    for row in rows:
        parameters = row.get('motion_contact_parameters', row.get('motion_reliability_parameters', []))
        force = row.get('motion_support_fraction', [None, None])[side]
        dt = row.get('control_dt_s')
        continuous = previous is None or (
            row['sequence'] == previous['sequence']+1 and
            0 < row['monotonic_ns']-previous['monotonic_ns'] <= 3_000_000)
        low = (row.get('motion_estimator_valid') and row.get('motion_support_valid') and
               len(parameters) >= 3 and finite(parameters[0]) and finite(force) and
               force < parameters[0] and finite(dt) and 0 < dt <= .02)
        if not continuous or not low:
            if duration:
                runs.append(duration)
            duration = 0.
        if low:
            duration += dt
        previous = row
    if duration:
        runs.append(duration)
    return distribution(runs)


def analyze(rows, pair_thresholds=(.2, .4, .6, .8), innovation_floors=(.2, .4, .6, .8)):
    if not rows:
        raise ValueError('Empty capture')
    if any(row.get('version', 0) < 9 for row in rows):
        raise ValueError('Threshold logs require telemetry version 9 or newer')
    timestamps = [row['monotonic_ns'] for row in rows]
    if any(b <= a for a, b in zip(timestamps, timestamps[1:])):
        raise ValueError('Source timestamps are not increasing; split restarts/duplicate streams')
    intervals_ms = [(b-a)*1e-6 for a, b in zip(timestamps, timestamps[1:])]
    sequences = [row['sequence'] for row in rows]
    if any(b <= a for a, b in zip(sequences, sequences[1:])):
        raise ValueError('Non-increasing sequence; split source restarts')
    source_gaps = sum(max(0, b-a-1) for a, b in zip(sequences, sequences[1:]))
    def counter_delta(name):
        values = [row.get(name) for row in rows]
        if not all(finite(value) for value in values):
            return None
        if any(b < a for a, b in zip(values, values[1:])):
            return None
        return values[-1]-values[0]
    warnings = []
    if not any(row.get('motion_estimator_valid') for row in rows):
        warnings.append('No valid estimator samples: raw-input inspection only; residual thresholds cannot be calibrated.')
    if source_gaps or (intervals_ms and max(intervals_ms) > 3.):
        warnings.append('Source gaps present: sub-millisecond peaks/event durations may be missing; do not interpolate them away.')
    if any(row.get('simulated') for row in rows):
        warnings.append('Contains simulated data; not evidence for physical threshold calibration.')
    if not any(row.get('wheel_prediction_gate_active', [False, False])[0] for row in rows):
        warnings.append('No active prediction gate. Version 10 uses fixed-noise fusion; residuals are diagnostics only.' if all(row['version'] >= 10 for row in rows) else 'No active inertial gate; cannot calibrate absolute prediction gating from this log.')
    if any(row.get('condition') == 'anomaly' for row in rows):
        warnings.append('Anomaly labels describe the experiment, not frame-level slip truth; exceedance is not detection accuracy.')
    groups = defaultdict(list)
    for row in rows:
        groups[(row.get('condition', 'unlabeled'), row.get('scenario', 'unknown'))].append(row)
    grouped = {}
    for (condition, scenario), samples in groups.items():
        signals = collect_signals(samples)
        entry = {'condition': condition, 'scenario': scenario, 'samples': len(samples),
                 'signals': signals, 'threshold_scan': threshold_scan(samples, pair_thresholds, innovation_floors),
                 'low_support_runs_s': {name: contact_loss_runs(samples, side) for side, name in enumerate(('left', 'right'))}}
        if all(row['version'] >= 10 for row in samples):
            entry.pop('threshold_scan')
            entry['residual_diagnostics_only'] = True
        # Quantile envelopes are candidates for historical normal-data review.
        if condition == 'normal' and any(row['version'] == 9 for row in samples) and not any(row.get('simulated') for row in samples):
            entry['normal_p99_9_envelopes'] = {
                name: stats.get('p99.9') for name, stats in signals.items()
                if name.endswith(('pair_abs_mps', 'prediction_abs_mps', 'prediction_ratio'))}
            entry['normal_p99_9_note'] = 'Empirical envelope only; inspect different speeds, reversals and spin directions before selecting thresholds.'
        grouped[f'{condition}:{scenario}'] = entry
    parameter_snapshots = sorted({tuple(row['motion_reliability_parameters']) for row in rows
                                  if row.get('motion_estimator_valid') and
                                  len(row.get('motion_reliability_parameters', [])) == 9 and
                                  all(finite(value) for value in row['motion_reliability_parameters'])})
    if len(parameter_snapshots) > 1:
        warnings.append('Actual threshold parameters changed within capture; compare separately before tuning.')
    imu_rows = [row for row in rows if row.get('imu_fresh')]
    repeated_imu = sum(a.get('imu_sequence') == b.get('imu_sequence')
                       for a, b in zip(imu_rows, imu_rows[1:]))
    return {'samples': len(rows), 'duration_s': (timestamps[-1]-timestamps[0])*1e-9,
            'input_timing': {
                'control_dt_s': distribution([row.get('control_dt_s') for row in rows]),
                'imu_age_us': distribution([row.get('imu_age_us') for row in rows]),
                'consecutive_repeated_imu_frames': repeated_imu,
                'left_motor_age_us': distribution([row.get('motor_feedback_age_us', [None]*6)[0] for row in rows]),
                'right_motor_age_us': distribution([row.get('motor_feedback_age_us', [None]*6)[1] for row in rows])},
            'source_interval_ms': distribution(intervals_ms), 'source_sequence_gaps': source_gaps,
            'source_queue_drops_delta': counter_delta('queue_drops'),
            'missed_releases_delta': counter_delta('missed_releases'),
            'recorder_queue_drops_last': rows[-1].get('recorder_queue_drops'),
            'invalid_estimator_samples': sum(not row.get('motion_estimator_valid') for row in rows),
            'left_status_counts': dict(Counter(str(row['wheel_speed_status'][0]) for row in rows)),
            'right_status_counts': dict(Counter(str(row['wheel_speed_status'][1]) for row in rows)),
            'actual_reliability_parameters': [dict(zip(RELIABILITY_PARAMETER_NAMES, values)) for values in parameter_snapshots],
            'actual_contact_parameters': [dict(zip(RELIABILITY_PARAMETER_NAMES[:3], values)) for values in sorted({
                tuple(row['motion_contact_parameters']) for row in rows
                if row.get('motion_estimator_valid') and len(row.get('motion_contact_parameters', [])) == 3
                and all(finite(value) for value in row['motion_contact_parameters'])})],
            'groups': grouped, 'warnings': warnings,
            'limitations': ['Threshold scans hold the logged prior fixed; they are NOT a KF replay with candidate parameters.',
                            'Accepted/rejected status is not ground truth and never filters the residual distributions.',
                            'No automatic edits to controller configuration.',
                            'Also inspect recorder summary: drops after the final written sample appear only there.']}


def write_events(rows, output):
    columns = ('monotonic_ns', 'sequence', 'side', 'status', 'evidence', 'contact',
               'prediction_gate_active', 'confidence', 'prediction_residual_mps', 'pair_residual_mps')
    with output.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        previous = [None, None]
        for row in rows:
            for side, name in enumerate(('left', 'right')):
                state = (row['wheel_speed_status'][side], row['wheel_speed_evidence'][side],
                         row['wheel_contact'][side], row['wheel_prediction_gate_active'][side])
                if state == previous[side]:
                    continue
                previous[side] = state
                writer.writerow(dict(zip(columns, (row['monotonic_ns'], row['sequence'], name,
                    *state, row['wheel_speed_confidence'][side],
                    row['wheel_speed_prediction_residual_mps'][side], row['wheel_speed_pair_residual_mps'][side]))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--pair-thresholds', type=float, nargs='+', default=[.2, .4, .6, .8])
    parser.add_argument('--innovation-floors', type=float, nargs='+', default=[.2, .4, .6, .8])
    args = parser.parse_args()
    if any(not math.isfinite(value) or value <= 0. for value in args.pair_thresholds+args.innovation_floors):
        parser.error('candidate thresholds must be finite and positive')
    with args.capture.open() as stream:
        rows = [json.loads(line) for line in stream if line.strip()]
    try:
        report = analyze(rows, args.pair_thresholds, args.innovation_floors)
    except (ValueError, KeyError, IndexError, TypeError) as error:
        raise SystemExit(f'Invalid/incomplete threshold capture: {error}')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir/'threshold_report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    write_events(rows, args.output_dir/'events.csv')
    print(f'{len(rows)} samples; {report["source_sequence_gaps"]} sequence gaps; report: {args.output_dir}')
    for warning in report['warnings']:
        print('WARNING:', warning)


if __name__ == '__main__':
    main()
