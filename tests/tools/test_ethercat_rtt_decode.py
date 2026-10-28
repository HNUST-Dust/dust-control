import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('decode', Path(__file__).resolve().parents[2]/'tools/ethercat_rtt_decode.py')
d = importlib.util.module_from_spec(spec)
spec.loader.exec_module(d)

class TraceContracts(unittest.TestCase):
    def fixture(self, valid=65535, seq=9, revision=32):
        header = d.HEADER.pack(0x31525444, 1, 168, 128, 1, 0, 24000000, 40, 7)
        # Hardware counter deliberately crosses 32-bit boundary.
        cycles = [2**32+240*i for i in range(16)]
        record = d.RECORD.pack(revision, valid, 11, 3, 123, 99, seq, 0, 0, *cycles)
        data = header+record+bytes(127*168)
        row = dict(boot='7', session='11', batch='3', publication='100', can_sequence='9', exchange='123')
        for i, s in enumerate(('built', 'encoded', 'output_copied', 'queued', 'tx', 'send_enter', 'send_return', 'receive_enter', 'rx', 'domain_processed', 'image_copied', 'decoded', 'bench_consumed')):
            row[s+'_ns'] = str(100000+i*1000)
        return data, row
    def test_complete_and_retained_publication(self):
        data,row = self.fixture()
        out = d.join(data,[row])[0]
        self.assertEqual(out['trace_status'],'ok')
        self.assertEqual(out['mcu_rx_map_to_esc_end_us'],'130.000')
        self.assertEqual(out['host_built_to_bench_consumed_us'],'12.000')
        row['exchange']='122'
        later=d.join(data,[row])[0]
        self.assertEqual(later['trace_status'],'ok')
        self.assertEqual(later['accepted_exchange_advance'],1)
    def test_v2_queue_stages_and_v1_compatibility(self):
        old,row=self.fixture()
        values=list(d.RECORD.unpack_from(old,d.HEADER.size))
        values[0]=36
        values[1]=(1<<18)-1
        cycles=values[9:]
        values += [cycles[10]+40,cycles[11]-40]
        data=d.HEADER.pack(0x31525444,2,184,128,1,0,24000000,40,7)
        data+=d.RECORD_V2.pack(*values)+bytes(127*184)
        out=d.join(data,[row])[0]
        self.assertEqual(out['trace_status'],'ok')
        self.assertEqual(out['mcu_can_enqueued_to_can_dequeued_us'],'6.667')
        self.assertEqual(out['mcu_can_dequeued_to_capture_us'],'1.667')
    def test_v3_snapshot_stage(self):
        old,row=self.fixture()
        values=list(d.RECORD.unpack_from(old,d.HEADER.size))
        values[0]=38
        values[1]=(1<<19)-1
        cycles=values[9:]
        values += [cycles[10]+40, cycles[11]-40, cycles[11]+120]
        data=d.HEADER.pack(0x31525444,3,192,128,1,0,24000000,40,7)
        data+=d.RECORD_V3.pack(*values)+bytes(127*192)
        out=d.join(data,[row])[0]
        self.assertEqual(out['trace_status'],'ok')
        self.assertEqual(out['mcu_capture_to_can_snapshot_us'],'5.000')
        self.assertEqual(out['mcu_can_snapshot_to_publish_us'],'5.000')

    def test_missing_mismatched_and_incomplete(self):
        data,row = self.fixture(valid=2047, seq=10)
        flags = d.join(data,[row])[0]['trace_status']
        self.assertIn('incomplete',flags)
        self.assertIn('can_sequence_mismatch',flags)
        row['boot']='8'
        self.assertIn('missing',d.join(data,[row])[0]['trace_status'])
    def test_reject_partial_dump_and_torn_record(self):
        data,row=self.fixture(revision=1)
        with self.assertRaises(ValueError): d.join(data,[row])
        with self.assertRaises(ValueError): d.read_trace(data[:-1])
    def test_duplicate_host_and_wrong_exchange(self):
        data,row=self.fixture()
        with self.assertRaises(ValueError): d.join(data,[row,row])
        row['exchange']='124'
        self.assertIn('exchange_mismatch',d.join(data,[row])[0]['trace_status'])

if __name__ == '__main__': unittest.main()
