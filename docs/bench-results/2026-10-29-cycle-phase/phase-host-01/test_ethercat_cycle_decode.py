import importlib.util
from pathlib import Path
import unittest
spec = importlib.util.spec_from_file_location('cycle_decode',
    Path(__file__).resolve().parents[2]/'tools/ethercat_cycle_decode.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

class CycleDecode(unittest.TestCase):
    def fixture(self):
        # Deliberately irregular host timings: actual iteration delta must come
        # from indices, not nearest(RTT/period). Final receive has no normal send.
        rows = [dict(iteration=i, command_batch=7, received_ns=t,
            receive_enter_ns=t-1000, previous_send_ns=t-120000,
            send_enter_ns=t+1000, send_return_ns=t+2000,
            wake_lateness_us=.1, publication=i, feedback_batch=i)
            for i,t in [(100,1000000),(101,1125000),(102,1300000)]]
        rows[-1]['send_enter_ns'] = rows[-1]['send_return_ns'] = 0
        sample = dict(batch=7, sample=0, exchange=100, rx_ns=1300000,
            tx_ns=1000500, send_enter_ns=1000800, send_return_ns=1002500,
            rtt_us=299.5)
        return [sample], rows
    def test_actual_iterations_and_last_old(self):
        samples, rows = self.fixture()
        r = mod.decode(samples, rows)[0]
        self.assertEqual(r['elapsed_iterations'], 2)
        self.assertEqual(r['last_old_to_response_poll_us'], 175)
        self.assertEqual(r['last_old_publication'], 101)
    def test_missing_iteration(self):
        samples, rows = self.fixture()
        with self.assertRaisesRegex(ValueError, 'missing or mismatched'):
            mod.decode(samples, [rows[0], rows[-1]])
    def test_wrong_response(self):
        samples, rows = self.fixture()
        samples[0]['rx_ns'] += 1
        with self.assertRaisesRegex(ValueError, 'timestamp mismatch'):
            mod.decode(samples, rows)
    def test_duplicate_cycle(self):
        samples, rows = self.fixture()
        with self.assertRaisesRegex(ValueError, 'strictly increase'):
            mod.decode(samples, [rows[0], rows[0], *rows[1:]])

if __name__ == '__main__':
    unittest.main()
