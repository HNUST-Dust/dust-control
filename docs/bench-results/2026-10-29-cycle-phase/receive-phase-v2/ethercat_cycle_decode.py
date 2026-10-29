#!/usr/bin/env python3
"""Join successful RTT samples with optional host-clock cycle telemetry."""
import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path

def decode(samples, cycles):
    groups = defaultdict(list)
    last = 0
    for row in cycles:
        i = int(row['iteration'])
        if i <= last:
            raise ValueError('cycle iterations must strictly increase')
        last = i
        if int(row['command_batch']):
            groups[int(row['command_batch'])].append(row)
    output = []
    for sample in samples:
        rows = groups.pop(int(sample['batch']), [])
        if len(rows) < 2:
            raise ValueError('missing command/response cycles')
        start, end = rows[0], rows[-1]
        a, b = int(start['iteration']), int(end['iteration'])
        if a != int(sample['exchange']) or b-a+1 != len(rows):
            raise ValueError('missing or mismatched pending iteration')
        if int(end['received_ns']) != int(sample['rx_ns']):
            raise ValueError('response cycle timestamp mismatch')
        if not (int(sample['tx_ns']) <= int(sample['send_enter_ns'])
                <= int(start['send_enter_ns']) <= int(start['send_return_ns'])
                <= int(sample['send_return_ns'])):
            raise ValueError('command send timestamps mismatch')
        before = rows[-2]
        output.append(dict(sample=int(sample['sample']), batch=int(sample['batch']),
            command_iteration=a, response_iteration=b, elapsed_iterations=b-a,
            rtt_us=float(sample['rtt_us']),
            response_wake_lateness_us=float(end['wake_lateness_us']),
            response_send_schedule_ns=int(end.get('send_schedule_ns', 0)),
            response_receive_call_us=(int(end['received_ns'])-int(end['receive_enter_ns']))/1000,
            response_previous_send_to_receive_us=(int(end['received_ns'])-int(end['previous_send_ns']))/1000,
            last_old_to_response_poll_us=(int(end['receive_enter_ns'])-int(before['receive_enter_ns']))/1000,
            last_old_publication=int(before['publication']),
            response_publication=int(end['publication']),
            last_old_feedback_batch=int(before['feedback_batch']),
            response_feedback_batch=int(end['feedback_batch'])))
    if groups:
        raise ValueError('unmatched cycles; require a complete successful run')
    return output

if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('samples'); p.add_argument('cycles'); p.add_argument('output')
    args = p.parse_args()
    with open(args.samples) as f, open(args.cycles) as g:
        rows = decode(list(csv.DictReader(f)), list(csv.DictReader(g)))
    if not rows:
        raise ValueError('empty run')
    with open(args.output, 'w') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    print(json.dumps({'samples':len(rows), 'actual_iteration_deltas':dict(Counter(
        r['elapsed_iterations'] for r in rows)),
        'note':'Host software observations only; not NIC arrival or ESC sampling timestamps.'}))
