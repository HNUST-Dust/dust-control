import importlib.util
import struct
import unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location('mcan_decode', Path(__file__).resolve().parents[2] / 'tools/ethercat_mcan_decode.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def data(records):
    header = m.HEADER.pack(0x3154434d, 1, m.RECORD.size, 128, len(records), 0, 24000000, 40, 7)
    return header + b''.join(m.RECORD.pack(*r) for r in records) + bytes((128-len(records))*m.RECORD.size)


def record(batch=1, seq=9, before=1200, rx=65530, counter=5, tscc=0xf0001):
    return (2, 1, 11, batch, seq, before-240, before, before+24, tscc, rx, counter, 1000000, 1, 1)


def row(batch=1, seq=9, callback=1440):
    return dict(boot='7', session='11', batch=str(batch), can_sequence=str(seq), mcu_can_rx_cycles=str(callback))


class Tests(unittest.TestCase):
    def test_wrap_and_cpu_intervals(self):
        r = m.join(data([record()]), [row()])[0]
        self.assertEqual(r['mcan_timing_status'], 'ok')
        self.assertEqual(r['mcan_rx_timestamp_age_us'], '176.000')
        self.assertEqual(r['mcan_cpu_irq_to_callback_us'], '20.000')
        self.assertEqual(r['mcan_cpu_sample_window_us'], '1.000')

    def test_matching_and_config(self):
        r = m.join(data([record()]), [row(seq=8)])[0]
        self.assertIn('can_sequence_mismatch', r['mcan_timing_status'])
        r = m.join(data([record(tscc=0)]), [row()])[0]
        self.assertIn('invalid_mcan_counter_config', r['mcan_timing_status'])
        self.assertNotIn('mcan_rx_timestamp_age_us', r)
        self.assertEqual(m.join(data([]), [row()])[0]['mcan_timing_status'], 'missing_mcan_record')

    def test_counter_rate(self):
        good = data([record(counter=5), record(batch=2, before=241200, counter=630)])
        rows = [row(), row(batch=2, callback=241440)]
        self.assertEqual(m.join(good, rows)[1]['mcan_counter_calibration_error_us'], '0.000')
        bad = data([record(counter=5), record(batch=2, before=241200, counter=6)])
        self.assertIn('mcan_counter_rate_mismatch', m.join(bad, rows)[1]['mcan_timing_status'])

    def test_reject_corruption_and_duplicates(self):
        for raw in [b'', data([record()])[:-1], data([record(), record()])]:
            with self.assertRaises(ValueError): m.read_timing(raw)
        with self.assertRaises(ValueError): m.join(data([record()]), [row(), row()])


if __name__ == '__main__':
    unittest.main()
