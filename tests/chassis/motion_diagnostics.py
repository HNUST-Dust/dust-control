"""Threshold-log correctness without ROS or hardware."""
import csv
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
from capture_motion_diagnostics import finite_json, sample_record, JsonlWriter
from analyze_motion_diagnostics import analyze, write_events


def sample(sequence=1):
    return {'version': 9, 'sequence': sequence, 'monotonic_ns': sequence*1_000_000,
            'control_dt_s': .001, 'queue_drops': 0, 'missed_releases': 0,
            'recorder_queue_drops': 0, 'condition': 'normal', 'scenario': 'flat_reverse',
            'simulated': False, 'imu_fresh': True, 'imu_raw_accel_g': [0., 0., 1.],
            'forward_acceleration_valid': True, 'forward_acceleration_mps2': 0.,
            'motion_estimator_valid': True, 'motion_prior_speed_variance': .01,
            'motion_wheel_valid': [True, True], 'motion_support_valid': True,
            'motion_support_fraction': [.5, .5], 'wheel_contact_known': [True, True],
            'wheel_contact': [True, True], 'wheel_low_support_time_s': [0., 0.],
            'wheel_pair_comparison_valid': [True, True],
            'wheel_prediction_gate_active': [True, True],
            'wheel_speed_prediction_residual_mps': [.1, -.1],
            'wheel_speed_pair_residual_mps': [.2, .2], 'wheel_prediction_ratio': [.25, .25],
            'wheel_speed_status': [4, 4], 'wheel_speed_evidence': [0, 0],
            'wheel_speed_confidence': [1., 1.],
            'motion_reliability_parameters': [.06, .12, .005, .1, .4, .4, 3., 3., .05]}


class ThresholdLogs(unittest.TestCase):
    def test_null_is_not_zero_and_invalid_samples_are_kept(self):
        class Message:
            header = SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=2))
            values = [float('nan'), float('inf'), -2.]
            motion_estimator_valid = False
            def get_fields_and_field_types(self):
                return dict.fromkeys(('header', 'values', 'motion_estimator_valid'))
        row = sample_record(Message(), 'flight', 'unlabeled', 123, 7)
        self.assertEqual(row['values'], [None, None, -2.])
        self.assertFalse(row['motion_estimator_valid'])
        self.assertEqual(row['recorder_queue_drops'], 7)
        json.dumps(row, allow_nan=False)
        self.assertIsNone(finite_json(float('nan')))

    def test_rejected_samples_are_not_removed_from_quantiles(self):
        rows = [sample(1), sample(2)]
        rows[1]['wheel_speed_status'] = [6, 6]
        rows[1]['wheel_speed_pair_residual_mps'] = [5., 5.]
        report = analyze(rows)
        group = report['groups']['normal:flat_reverse']
        self.assertEqual(group['signals']['left_pair_abs_mps']['max'], 5.)
        self.assertEqual(group['threshold_scan']['pair'][1]['exceedance_fraction'], .5)
        self.assertEqual(report['left_status_counts'], {'4': 1, '6': 1})

    def test_missing_comparison_not_a_zero_measurement(self):
        rows = [sample(1), sample(2)]
        rows[1]['wheel_pair_comparison_valid'] = [False, False]
        rows[1]['wheel_speed_pair_residual_mps'] = [None, None]
        stats = analyze(rows)['groups']['normal:flat_reverse']['signals']['left_pair_abs_mps']
        self.assertEqual(stats['samples'], 1)
        self.assertEqual(stats['mean'], .2)

    def test_gaps_split_contact_runs_and_emit_quality_warning(self):
        rows = [sample(1), sample(2), sample(5)]
        for row in rows:
            row['motion_support_fraction'] = [0., .5]
        report = analyze(rows)
        self.assertEqual(report['source_sequence_gaps'], 2)
        stats = report['groups']['normal:flat_reverse']['low_support_runs_s']['left']
        self.assertEqual(stats['samples'], 2)
        self.assertEqual(stats['max'], .002)
        self.assertTrue(report['warnings'])

    def test_invalid_and_simulated_data_have_no_tuning_envelope(self):
        row = sample()
        row['simulated'] = True
        row['motion_estimator_valid'] = False
        row['left_support_force_n'] = 54.
        report = analyze([row])
        self.assertEqual(report['groups']['normal:flat_reverse']['signals']['left_support_force_n']['max'], 54.)
        self.assertEqual(report['invalid_estimator_samples'], 1)
        self.assertNotIn('normal_p99_9_envelopes', report['groups']['normal:flat_reverse'])
        with self.assertRaises(ValueError):
            analyze([sample(2), sample(1)])
        row['version'] = 8
        with self.assertRaises(ValueError):
            analyze([row])

    def test_events_and_async_disk_writer(self):
        rows = [sample(1), sample(2), sample(3)]
        rows[2]['wheel_speed_status'][0] = 6
        rows[2]['wheel_speed_evidence'][0] = 64
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'samples.jsonl'
            writer = JsonlWriter(path)
            for row in rows:
                writer.submit(row)
            writer.close()
            self.assertIsNone(writer.error)
            self.assertEqual(writer.written, 3)
            self.assertEqual(len(path.read_text().splitlines()), 3)
            events = Path(directory)/'events.csv'
            write_events(rows, events)
            with events.open() as stream:
                events = list(csv.DictReader(stream))
            self.assertEqual(len(events), 3) # initial L/R plus L transition
            self.assertEqual(events[-1]['evidence'], '64')
            self.assertEqual(events[-1]['side'], 'left')


if __name__ == '__main__':
    unittest.main()
