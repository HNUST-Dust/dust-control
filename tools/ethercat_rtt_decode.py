#!/usr/bin/env python3
"""Join bounded DTR1 firmware traces to host CSV. Never subtract independent clocks."""
import argparse
import csv
import struct
from pathlib import Path

STAGES = ('esc_read_begin', 'esc_read_end', 'rx_map', 'process', 'process_done',
          'accept', 'enqueue', 'submit', 'submitted',
          'tx_done', 'can_rx', 'capture', 'publish', 'input_map', 'esc_begin', 'esc_end', 'can_enqueued', 'can_dequeued', 'can_snapshot')
HEADER = struct.Struct('<8IQ')
RECORD = struct.Struct('<IIQIIIIii16Q')
RECORD_V2 = struct.Struct('<IIQIIIIii18Q')
RECORD_V3 = struct.Struct('<IIQIIIIii19Q')
# TX completion callback and RX callback may run in either order after IRQ latency.
PAIRS = [('esc_read_begin', 'esc_read_end'), ('esc_read_end', 'rx_map'),
         ('rx_map', 'process'), ('process', 'process_done'),
         ('process_done', 'accept'), ('accept', 'enqueue'),
         ('enqueue', 'submit'), ('submit', 'submitted'), ('submit', 'tx_done'),
         ('submit', 'can_rx'), ('can_rx', 'capture'), ('capture', 'publish'),
         ('can_rx', 'can_enqueued'), ('can_enqueued', 'can_dequeued'),
         ('can_dequeued', 'capture'), ('capture', 'can_snapshot'),
         ('can_snapshot', 'publish'),
         ('publish', 'input_map'), ('input_map', 'esc_begin'), ('esc_begin', 'esc_end'),
         ('rx_map', 'esc_end')]
HOST_PAIRS = [('built', 'encoded'), ('encoded', 'output_copied'),
              ('output_copied', 'queued'), ('queued', 'tx'), ('tx', 'send_enter'),
              ('send_enter', 'send_return'),
              ('receive_enter', 'rx'), ('rx', 'domain_processed'),
              ('domain_processed', 'image_copied'), ('image_copied', 'decoded'),
              ('decoded', 'bench_consumed'), ('built', 'bench_consumed')]

def read_trace(data):
    if len(data) < HEADER.size:
        raise ValueError('truncated trace header')
    magic, version, size, capacity, count, overflow, hz, header, boot = HEADER.unpack_from(data)
    formats = {1: (RECORD, STAGES[:16]), 2: (RECORD_V2, STAGES[:18]),
               3: (RECORD_V3, STAGES)}
    if version not in formats:
        raise ValueError('unsupported trace ABI')
    record_format, stages = formats[version]
    if (magic, size, header) != (0x31525444, record_format.size, HEADER.size):
        raise ValueError('unsupported trace ABI')
    if not hz or not boot or capacity != 128 or count > capacity:
        raise ValueError('invalid trace metadata')
    if len(data) != header + capacity * size:
        raise ValueError('wrong dump size (use symbol size from the matching ELF)')
    records = {}
    for i in range(count):
        rev, valid, session, batch, exchange, publication, seq, submit, done, *cycles = record_format.unpack_from(data, header+i*size)
        if rev & 1 or valid & ~((1 << len(stages))-1):
            raise ValueError(f'inconsistent record {i}; dump after halting the CPU')
        key = (boot, session, batch)
        if key in records:
            raise ValueError(f'duplicate command key {key}')
        records[key] = dict(valid=valid, exchange=exchange, publication=publication,
                            can_sequence=seq, submit_result=submit, done_result=done,
                            stage_count=len(stages), cycles=dict(zip(stages, cycles)))
    return hz, overflow, records

def join(data, host):
    hz, overflow, records = read_trace(data)
    seen = set()
    output = []
    for row in host:
        key = tuple(int(row[k]) for k in ('boot', 'session', 'batch'))
        if key in seen:
            raise ValueError(f'duplicate host command {key}')
        seen.add(key)
        out = dict(row)
        out['trace_overflow'] = overflow
        record = records.get(key)
        flags = []
        if record is None:
            flags.append('missing_firmware_record')
        else:
            out.update({f'mcu_{k}': record[k] for k in ('valid', 'exchange', 'publication', 'can_sequence', 'submit_result', 'done_result')})
            out.update({f'mcu_{k}_cycles': v for k, v in record['cycles'].items()})
            valid = record['valid']
            if valid != (1 << record['stage_count'])-1:
                flags.append('incomplete_firmware_record')
            if record['submit_result'] or record['done_result']:
                flags.append('can_driver_error')
            if (valid & (1 << STAGES.index('can_rx'))) and int(row['can_sequence']) != record['can_sequence']:
                flags.append('can_sequence_mismatch')
            advance = (record['exchange']-int(row['exchange'])) & 0xffffffff
            out['accepted_exchange_advance'] = advance
            if advance >= 0x80000000:
                flags.append('command_exchange_mismatch')
            # The master can first observe a later publication of the same retained CAN snapshot.
            if valid & (1 << STAGES.index('publish')) and ((int(row['publication'])-record['publication']) & 0xffffffff) >= 0x80000000:
                flags.append('publication_precedes_feedback')
            for a, b in PAIRS:
                if all(s in record['cycles'] and valid & (1 << STAGES.index(s)) for s in (a, b)):
                    delta = record['cycles'][b]-record['cycles'][a]
                    if delta < 0:
                        flags.append(f'nonmonotonic_{a}_{b}')
                    else:
                        out[f'mcu_{a}_to_{b}_us'] = f'{delta*1e6/hz:.3f}'
        for a, b in HOST_PAIRS:
            delta = int(row[b+'_ns'])-int(row[a+'_ns'])
            if delta < 0:
                flags.append(f'host_nonmonotonic_{a}_{b}')
            else:
                out[f'host_{a}_to_{b}_us'] = f'{delta/1000:.3f}'
        out['trace_status'] = '|'.join(flags) or 'ok'
        output.append(out)
    return output

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('firmware_bin', type=Path)
    parser.add_argument('host_csv', type=Path)
    parser.add_argument('output_csv', type=Path)
    args = parser.parse_args()
    with args.host_csv.open(newline='') as stream:
        rows = join(args.firmware_bin.read_bytes(), list(csv.DictReader(stream)))
    if not rows:
        parser.error('host CSV contains no completed samples')
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with args.output_csv.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    good = sum(row['trace_status'] == 'ok' for row in rows)
    print(f'joined={len(rows)} complete={good} flagged={len(rows)-good}; independent clocks, no cross-clock subtraction')
    return 0 if good == len(rows) else 1

if __name__ == '__main__':
    raise SystemExit(main())
