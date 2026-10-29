#!/usr/bin/env python3
"""Reproduce the 125-us phase review using archived CSVs only.
Never subtract a host timestamp from an MCU timestamp.
"""
from pathlib import Path
import collections
import csv
import json
import math
import statistics

ROOT = Path(__file__).resolve().parents[1]
PERIOD_US = 125

def stats(values):
    values = sorted(values)
    return dict(n=len(values), mean=statistics.mean(values), minimum=values[0],
                median=statistics.median(values),
                p95=values[math.ceil(.95 * len(values))-1], maximum=values[-1])

def analyze(path):
    rows = list(csv.DictReader(path.open()))
    bands = collections.defaultdict(list)
    for row in rows:
        # Nearest nominal-cycle band, not a measured cycle index or deadline.
        band = math.floor(float(row['rtt_us']) / PERIOD_US + .5)
        bands[band].append(row)
    residual = [float(r['rtt_us']) - math.floor(float(r['rtt_us']) /
                PERIOD_US + .5) * PERIOD_US for r in rows]
    result = dict(samples=len(rows), residual_us=stats(residual), bands={})
    for band, group in sorted(bands.items()):
        item = dict(count=len(group), fraction=len(group)/len(rows),
                    rtt_us=stats([float(r['rtt_us']) for r in group]))
        if 'mcu_rx_map_cycles' in group[0]:
            assert all(r['trace_status'] == 'ok' for r in group)
            for begin, end in [('rx_map','can_rx'), ('can_rx','esc_end'),
                               ('rx_map','esc_end'), ('capture','can_snapshot')]:
                # All these archived v7 traces report a 24MHz MCU timer.
                item[begin+'_to_'+end+'_us'] = stats([
                    (int(r['mcu_'+end+'_cycles']) -
                     int(r['mcu_'+begin+'_cycles']))/24 for r in group])
        result['bands'][str(band)] = item
    return result

if __name__ == '__main__':
    output = {'period_us': PERIOD_US, 'traceoff': {}, 'traceon': {}}
    for run in ['a1','b1','b2','a2']:
        path = ROOT/'2026-10-29-fast-notice-abba'/('abba-off-'+run[0].upper())/(
            'dm-rtt-125us-abba-off-'+run+'.csv')
        output['traceoff'][run] = analyze(path)
    for tag, folder in [('baseline','2026-10-29-ssc-phase/ssc-v7-01'),
                        ('late','2026-10-29-input-late-refresh/late-v7-01'),
                        ('fast','2026-10-29-input-fast-notice/fast-v7-01')]:
        output['traceon'][tag] = analyze(ROOT/folder/'decoded.csv')
    dest = Path(__file__).with_name('summary.json')
    dest.write_text(json.dumps(output, indent=2)+'\n')
    for name, result in output['traceoff'].items():
        print(name, {b: v['count'] for b, v in result['bands'].items()},
              'residual us', result['residual_us'])
