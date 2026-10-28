#!/usr/bin/env python3
"""Validate and summarize four fixed-condition ABBA DM-disable RTT runs.

Nearest-rank quantiles match ethercat_dm_rtt.cpp. This summarizes measured
samples, not a long-term latency bound or an independent-sample significance test.
"""
import argparse
import csv
import json
import math
import statistics
from pathlib import Path


def stats(rows):
    values = [(int(r['rx_ns'])-int(r['tx_ns']))/1000 for r in rows]
    ordered = sorted(values)
    def quantile(q):
        return ordered[math.ceil(q*len(ordered))-1]
    return dict(samples=len(values), mean_us=math.fsum(values)/len(values),
                min_us=ordered[0], p50_us=quantile(.5), p95_us=quantile(.95),
                p99_us=quantile(.99), max_us=ordered[-1],
                stddev_us=statistics.pstdev(values),
                over_1ms=sum(v > 1000 for v in values),
                over_1ms_percent=100*sum(v > 1000 for v in values)/len(values),
                nearest_125us_bins={str(k):sum(round(v/125)*125 == k for v in values)
                                    for k in sorted(set(round(v/125)*125 for v in values))})


def summarize(directory, manifest):
    run = manifest['run_id']
    expected = ['multi', 'single', 'single', 'multi']
    if manifest['variants'] != expected or manifest['samples_per_phase'] != 1000:
        raise ValueError('expected the prepared four-phase/1000-sample protocol')
    groups, phases, boots = {'multi': [], 'single': []}, [], set()
    for phase, variant in enumerate(expected, 1):
        prefix = directory/f'dm-rtt-125us-{run}-p{phase}-{variant}'
        with prefix.with_suffix('.csv').open(newline='') as source:
            rows = list(csv.DictReader(source))
        log = prefix.with_suffix('.log').read_text()
        for required in ['PASS commands=1000 timeouts=0 WKC_misses=0;',
                         'compiler_optimized=1', 'CPU=7', 'scheduler=FIFO40',
                         'mlock=1', 'PDO Rx=96 Tx=400 signature=3858229428 cycle_us=125',
                         'command_interval_us=10000 timeout_us=8000',
                         'restored l1_aspm=1', 'restored l1_1_aspm=1',
                         'restored l1_1_pcipm=1']:
            if required not in log:
                raise ValueError(f'phase {phase}: condition missing: {required}')
        if len(rows) != 1000 or [int(r['batch']) for r in rows] != list(range(1, 1001)):
            raise ValueError(f'phase {phase}: incomplete or inconsistent command sequence')
        identities = {(int(r['boot']), int(r['session'])) for r in rows}
        if len(identities) != 1 or not all(next(iter(identities))):
            raise ValueError(f'phase {phase}: inconsistent boot/session')
        boot = int(rows[0]['boot'])
        if boot in boots:
            raise ValueError('firmware was not freshly rebooted between phases')
        boots.add(boot)
        for r in rows:
            measured = (int(r['rx_ns'])-int(r['tx_ns']))/1000
            if measured <= 0 or abs(measured-float(r['rtt_us'])) > .0011:
                raise ValueError('invalid host RTT')
        groups[variant].extend(rows)
        phases.append(dict(phase=phase, variant=variant, boot=boot,
                           session=int(rows[0]['session']), **stats(rows)))
    pooled = {variant:stats(rows) for variant, rows in groups.items()}
    return dict(run_id=run, phases=phases, pooled=pooled,
                mean_difference_us=pooled['multi']['mean_us']-pooled['single']['mean_us'],
                mean_increase_percent=100*(pooled['multi']['mean_us']/pooled['single']['mean_us']-1),
                scope='125 us PDO, 10 ms sequential disable commands; no full mixed-load or hard-RT claim')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('data_directory', type=Path)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    summary = summarize(args.data_directory, json.loads(args.manifest.read_text()))
    args.output.write_text(json.dumps(summary, indent=2)+'\n')
    for phase in summary['phases']:
        print(f"phase={phase['phase']} {phase['variant']} n={phase['samples']} mean={phase['mean_us']:.3f} p99={phase['p99_us']:.3f} max={phase['max_us']:.3f} over1ms={phase['over_1ms']}")
    print(f"pooled mean difference={summary['mean_difference_us']:.3f} us ({summary['mean_increase_percent']:.3f}%)")


if __name__ == '__main__':
    main()
