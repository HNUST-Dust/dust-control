#!/usr/bin/env python3
"""Offline analysis of captured LQR14 telemetry; no hardware access or tuning."""
import argparse
import json
from pathlib import Path

import numpy as np

STATE_NAMES = ['position', 'speed', 'yaw', 'yaw_rate', 'left_theta', 'left_theta_rate',
               'right_theta', 'right_theta_rate', 'pitch', 'pitch_rate', 'roll',
               'roll_rate', 'height', 'height_rate']


def stats(x):
    x = np.asarray(x)
    x = x[np.isfinite(x)]
    if not len(x):
        return None
    return {'mean': float(np.mean(x)), 'rms': float(np.sqrt(np.mean(x*x))),
            'min': float(np.min(x)), 'max': float(np.max(x))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    rows = [json.loads(line) for line in args.capture.open() if line.strip()]
    if len(rows) < 100:
        raise SystemExit('Insufficient samples')
    def column(name):
        return np.array([row[name] for row in rows], dtype=float)
    time_s = (column('monotonic_ns') - rows[0]['monotonic_ns']) * 1e-9
    valid = np.array([row['capture_eligible'] for row in rows])
    if np.any(np.diff(time_s) <= 0):
        raise SystemExit('Non-increasing source timestamps; split capture before analysis')
    if np.count_nonzero(valid) < 100:
        raise SystemExit("Insufficient eligible samples; no controller diagnosis possible")
    duration = float(time_s[-1])
    report = {'duration_s': duration, 'samples': len(rows),
              'eligible_fraction': float(np.mean(valid)),
              'median_source_interval_ms': float(np.median(np.diff(time_s))*1000),
              'max_source_interval_ms': float(np.max(np.diff(time_s))*1000),
              'sequence_gaps': int(np.maximum(0, np.diff(column('sequence'))-1).sum()),
              'missed_releases_delta': float(column('missed_releases')[-1]-column('missed_releases')[0]),
              'queue_drops_delta': float(column('queue_drops')[-1]-column('queue_drops')[0])}
    keys = ['raw_body_speed_mps', 'estimated_body_speed_mps', 'target_body_speed_mps',
            'remote_chassis_x', 'body_position_error_m', 'body_position_error_unclamped_m',
            'pitch_deg', 'pitch_reference_deg', 'left_theta_deg', 'right_theta_deg', 'theta_reference_deg']
    report['signals'] = {key: stats(column(key)[valid]) for key in keys}
    report['speed_filter_difference'] = stats((column('estimated_body_speed_mps')-column('raw_body_speed_mps'))[valid])
    report['position_clipped_fraction'] = float(np.mean(np.abs(column('body_position_error_m')[valid]-column('body_position_error_unclamped_m')[valid]) > 1e-7)) if valid.any() else None
    terms = {}
    for side in ('left', 'right'):
        contributions = column(f'lqr14_{side}_wheel_contribution_nm')
        requested = column(f'lqr14_{side}_wheel_requested_nm')
        allocated = column(f'lqr14_{side}_wheel_allocated_nm')
        command = column(f'{side}_wheel_output_command_nm')
        terms[side] = {'contributions': {name: stats(contributions[valid, j]) for j, name in enumerate(STATE_NAMES)},
                       'sum_minus_request': stats((contributions.sum(axis=1)-requested)[valid]),
                       'requested': stats(requested[valid]), 'allocated': stats(allocated[valid]),
                       'output_command': stats(command[valid]),
                       'allocation_changed_fraction': float(np.mean(np.abs(requested[valid]-allocated[valid]) > 1e-5)),
                       'blend_changed_fraction': float(np.mean(np.abs(command[valid]-allocated[valid]) > 1e-5))}
    report['wheel_torques'] = terms
    # Frequency estimate only if the complete window is eligible. Resample timestamps,
    # never treat ROS publication gaps as a fixed 1kHz sample stream.
    uniform = np.arange(0, duration, .01)
    if np.all(valid) and duration >= 12:
        speed = np.interp(uniform, time_s, column('estimated_body_speed_mps'))
        speed -= np.mean(speed)
        frequency = np.fft.rfftfreq(len(speed), .01)
        amplitude = np.abs(np.fft.rfft(speed*np.hanning(len(speed))))
        band = (frequency >= .1) & (frequency <= 3.)
        f = float(frequency[np.where(band)[0][np.argmax(amplitude[band])]])
        report['oscillation_frequency_hz'] = f
        report['frequency_bin_width_hz'] = float(1/(len(speed)*.01))
        report['oscillation_period_s'] = 1/f
    else:
        report['frequency_not_estimated'] = 'Window shorter than12s or controller/zero-command eligibility interrupted'
    # Exclude recovery-reference transients, then fit the dominant low-frequency
    # oscillation with an offset/trend rather than claiming FFT-bin precision.
    if np.all(valid) and duration >= 16:
        mask = time_s >= 4.
        steady_t = time_s[mask]
        velocity = column('estimated_body_speed_mps')[mask]
        frequencies = np.linspace(.15, .45, 1201)
        def basis(f):
            return np.column_stack([np.ones(len(steady_t)), steady_t-steady_t.mean(),
                                    np.cos(2*np.pi*f*steady_t), np.sin(2*np.pi*f*steady_t)])
        costs = []
        for f in frequencies:
            matrix = basis(f)
            coefficients = np.linalg.lstsq(matrix, velocity, rcond=None)[0]
            costs.append(np.mean((velocity-matrix@coefficients)**2))
        fitted_f = float(frequencies[np.argmin(costs)])
        matrix = basis(fitted_f)
        def phasor(values):
            coefficients = np.linalg.lstsq(matrix, values[mask], rcond=None)[0]
            return coefficients[2]-1j*coefficients[3]
        raw = phasor(column('raw_body_speed_mps'))
        estimated = phasor(column('estimated_body_speed_mps'))
        phase = float(np.angle(estimated/raw)) if abs(raw) > 1e-9 else None
        report['steady_segment'] = {
            'start_s': 4., 'sinusoid_fit_frequency_hz': fitted_f,
            'fit_period_s': 1/fitted_f,
            'speed_sine_amplitude_mps': float(abs(estimated)),
            'raw_to_estimated_phase_deg': None if phase is None else float(np.rad2deg(phase)),
            'equivalent_filter_delay_ms': None if phase is None else -phase/(2*np.pi*fitted_f)*1000,
            'position_clipped_fraction': float(np.mean(np.abs(column('body_position_error_m')[mask]-column('body_position_error_unclamped_m')[mask]) > 1e-7)),
            'note': 'Single-sinusoid fit in 0.15-0.45Hz band; not system identification or proof of cause'}
    report['limitations'] = ['No measured wheel torque/current in this telemetry',
                             'Allocation changes may be yaw allocation rather than saturation',
                             'Correlation and periodicity do not establish causal gain errors',
                             'Small requests alone cannot prove motor deadzone or friction']
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir/'analysis.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    # Standalone figure for reviewing an entire capture without interactive panel scaling.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(4, 2, figsize=(16, 12), sharex=True)
    panels = [('speed (m/s)', ['raw_body_speed_mps','estimated_body_speed_mps','target_body_speed_mps']),
              ('position error (m)', ['body_position_error_m','body_position_error_unclamped_m']),
              ('theta (deg)', ['left_theta_deg','right_theta_deg','theta_reference_deg']),
              ('pitch (deg)', ['pitch_deg','pitch_reference_deg'])]
    for ax, (title, fields) in zip(axes.flat, panels):
        for field in fields:
            ax.plot(time_s, column(field), label=field, lw=.8)
        ax.set_title(title)
    for index, side in enumerate(('left','right')):
        ax = axes[2,index]
        contribution = column(f'lqr14_{side}_wheel_contribution_nm')
        for name, indices in [('position',[0]),('speed',[1]),('theta',[4,6]),('theta_rate',[5,7]),('pitch',[8]),('pitch_rate',[9]),('remaining',[2,3,10,11,12,13])]:
            ax.plot(time_s, contribution[:,indices].sum(axis=1), label=name, lw=.8)
        ax.set_title(side+' wheel feedback contributions (Nm)')
        ax = axes[3,index]
        for field in [f'lqr14_{side}_wheel_requested_nm',f'lqr14_{side}_wheel_allocated_nm',f'{side}_wheel_output_command_nm']:
            ax.plot(time_s, column(field), label=field, lw=.8)
        ax.set_title(side+' wheel torque commands (Nm), NOT measured torque')
    for ax in axes.flat:
        ax.grid(alpha=.3)
        ax.legend(fontsize=7, loc='upper right')
        ax.set_xlabel('source time since capture start (s)')
    fig.tight_layout()
    fig.savefig(args.output_dir/'balance-diagnostics.png', dpi=160)
    plt.close(fig)
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
