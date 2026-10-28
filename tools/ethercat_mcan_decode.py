#!/usr/bin/env python3
"""Join optional MCT1 MCAN diagnostics to the existing staged RTT CSV.

CPU intervals use only CPU cycles. RX age uses only the 16-bit MCAN counter.
Counter age is modulo one wrap (1.048576 s at 1 Mbps, TCP=16), includes
reception duration, and is not an absolute timestamp in the CPU clock domain.
"""
import argparse
import csv
import struct
from pathlib import Path

HEADER = struct.Struct('<8IQ')
RECORD = struct.Struct('<IIQII3Q6I')


def read_timing(data):
    if len(data) < HEADER.size:
        raise ValueError('truncated MCT1 header')
    magic, version, size, capacity, count, overflow, hz, header, boot = HEADER.unpack_from(data)
    if (magic, version, size, capacity, header) != (0x3154434d, 1, RECORD.size, 128, HEADER.size):
        raise ValueError('unsupported MCT1 ABI')
    if not hz or not boot or count > capacity or overflow or len(data) != header + capacity * size:
        raise ValueError('invalid MCT1 metadata or dump size')
    records = {}
    for index in range(count):
        revision, valid, session, batch, seq, irq, before, after, tscc, rx, counter, bitrate, fifo, flags = RECORD.unpack_from(data, header + index * size)
        if revision & 1 or valid not in (0, 1):
            raise ValueError('inconsistent MCT1 record; halt after master exits')
        if not valid:
            continue
        key = (boot, session, batch)
        if key in records:
            raise ValueError('duplicate MCT1 command key')
        records[key] = dict(sequence=seq, irq=irq, before=before, after=after,
                            tscc=tscc, rx=rx, counter=counter, bitrate=bitrate,
                            fifo=fifo, flags=flags)
    return hz, records


def join(data, rows):
    hz, records = read_timing(data)
    output, seen = [], set()
    previous = None
    for row in rows:
        out = dict(row)
        key = tuple(int(row[k]) for k in ('boot', 'session', 'batch'))
        if key in seen:
            raise ValueError('duplicate staged command key')
        seen.add(key)
        record = records.get(key)
        flags = []
        if record is None:
            flags.append('missing_mcan_record')
        else:
            out.update({f'mcan_{k}': v for k, v in record.items()})
            if int(row['can_sequence']) != record['sequence']:
                flags.append('can_sequence_mismatch')
            callback = int(row['mcu_can_rx_cycles'])
            if not record['irq'] <= record['before'] <= record['after'] <= callback:
                flags.append('nonmonotonic_cpu_timing')
            else:
                out['mcan_cpu_irq_to_callback_us'] = f"{(callback - record['irq']) * 1e6 / hz:.3f}"
                out['mcan_cpu_irq_to_sample_us'] = f"{(record['before'] - record['irq']) * 1e6 / hz:.3f}"
                out['mcan_cpu_sample_window_us'] = f"{(record['after'] - record['before']) * 1e6 / hz:.3f}"
            if record['tscc'] & 3 != 1 or not record['bitrate'] or max(record['rx'], record['counter']) > 65535:
                flags.append('invalid_mcan_counter_config')
            else:
                tick_us = (((record['tscc'] >> 16) & 15) + 1) * 1e6 / record['bitrate']
                out['mcan_tick_us'] = f'{tick_us:.6f}'
                out['mcan_counter_wrap_us'] = f'{tick_us * 65536:.3f}'
                out['mcan_rx_timestamp_age_us'] = f"{((record['counter'] - record['rx']) & 65535) * tick_us:.3f}"
                if record['fifo'] != 0xffffffff:
                    out['mcan_fifo_fill_before_read'] = record['fifo'] & 127
                if previous is not None and previous[0] == key[0]:
                    prev = previous[1]
                    if (prev['tscc'], prev['bitrate']) == (record['tscc'], record['bitrate']):
                        cpu_us = (record['before'] - prev['before']) * 1e6 / hz
                        if 0 < cpu_us < tick_us * 65536:
                            counter_us = ((record['counter'] - prev['counter']) & 65535) * tick_us
                            error = counter_us - cpu_us
                            out['mcan_counter_calibration_error_us'] = f'{error:.3f}'
                            if abs(error) > max(2 * tick_us, 0.02 * cpu_us):
                                flags.append('mcan_counter_rate_mismatch')
                previous = (key[0], record)
        out['mcan_timing_status'] = '|'.join(flags) or 'ok'
        output.append(out)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('timing_bin', type=Path)
    parser.add_argument('staged_csv', type=Path)
    parser.add_argument('output_csv', type=Path)
    args = parser.parse_args()
    with args.staged_csv.open(newline='') as source:
        output = join(args.timing_bin.read_bytes(), list(csv.DictReader(source)))
    if not output:
        parser.error('no staged rows')
    columns = list(dict.fromkeys(k for row in output for k in row))
    with args.output_csv.open('w', newline='') as dest:
        writer = csv.DictWriter(dest, fieldnames=columns, lineterminator='\n')
        writer.writeheader()
        writer.writerows(output)
    print(f"joined={len(output)} flagged={sum(r['mcan_timing_status'] != 'ok' for r in output)}; MCAN age is modulo counter wrap, independent clocks")


if __name__ == '__main__':
    main()
