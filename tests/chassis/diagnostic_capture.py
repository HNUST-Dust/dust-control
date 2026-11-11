"""Regression for actual ROS fixed-array storage types; no ROS runtime required."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
from capture_balance_diagnostics import json_native, serialize_message


class Message:
    header = SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=2))
    motor_target_mode = np.array([0, 1, 2, 3, 4, 1], dtype=np.uint8)
    wheel_current_feedback_raw = np.array([-5., 14.])

    def get_fields_and_field_types(self):
        return dict.fromkeys(['header', 'motor_target_mode', 'wheel_current_feedback_raw'])


class CaptureSerialization(unittest.TestCase):
    def test_fixed_arrays_and_signed_values(self):
        row = json.loads(serialize_message(Message(), False))
        self.assertEqual(row['motor_target_mode'], [0, 1, 2, 3, 4, 1])
        self.assertEqual(row['wheel_current_feedback_raw'], [-5., 14.])
        self.assertEqual(row['stamp_ns'], 1000000002)
        self.assertIs(row['capture_eligible'], False)
        self.assertEqual(json_native([np.uint8(255), np.int16(-100)]), [255, -100])

    def test_unknown_type_fails_explicitly(self):
        with self.assertRaises(TypeError):
            json_native(object())


if __name__ == '__main__':
    unittest.main()
