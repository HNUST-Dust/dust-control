#!/usr/bin/env python3
"""Decode saved DC owner timing profiles without touching the target."""
import argparse
import json
import struct
from pathlib import Path

NAMES = ["SSCother", "Tick", "CANreceive", "ProcessScope", "IOscope",
         "PublishScope", "Mapping", "ESCwrite", "IOchecks", "CANtxScope",
         "UARTtx", "CoreProcess", "Dispatch", "Capture", "Encode",
         "TxPrepare", "DriverValidate", "DriverAdmit", "DriverState",
         "DriverMessageRAM", "DriverRequest", "DriverRelease"]

def decode(directory):
    b = (directory / "dust_sync0_phase_stats.bin").read_bytes()
    version, count, stages, max_work, late, _ = struct.unpack_from("<6I", b)
    if version not in (1, 2, 3) or stages > len(NAMES) or not count:
        raise ValueError("Unsupported or empty profile")
    offset = 24
    def take(kind):
        nonlocal offset
        values = struct.unpack_from("<" + str(stages) + kind, b, offset)
        offset += struct.calcsize(kind) * stages
        return values
    total = take("Q")
    worker = take("Q") if version >= 2 else [0] * stages
    maximum, worst = take("I"), take("I")
    worker_worst = take("I") if version >= 2 else [0] * stages
    worker_max = take("I") if version >= 2 else [0] * stages
    if sum(worst) != max_work:
        raise ValueError("Worst-cycle phases do not sum to handler time")
    result = dict(version=version, services=count, mean_us=sum(total)/count/24,
                  max_us=max_work/24, worst_lateness_us=late/24, stages=[])
    for i in range(stages):
        result["stages"].append(dict(name=NAMES[i], mean_us=total[i]/count/24,
            max_us=maximum[i]/24, worst_us=worst[i]/24,
            worker_mean_us=worker[i]/count/24,
            worker_at_max_us=worker_max[i]/24, worker_worst_us=worker_worst[i]/24))
    slow_path = directory / "dust_sync0_slow_cycles.bin"
    if slow_path.exists():
        raw = slow_path.read_bytes()
        h = struct.unpack_from("<12I", raw)
        if h[0] != 1 or h[2] > 4 or h[1] != count or sum(h[4:11]) != count:
            raise ValueError("Inconsistent slow-cycle histogram")
        result["histogram_us"] = dict(zip(
            ["lt100", "100to125", "125to150", "150to200", "200to250", "250to500", "ge500"], h[4:11]))
        result["worker_overlap_cycles"] = h[3]
        records = []
        for i in range(h[2]):
            values = struct.unpack_from("<" + str(3 + 2*stages) + "I", raw,
                                        48 + i*(3+2*stages)*4)
            ordinal, work, lateness = values[:3]
            phases = values[3:3+stages]
            if sum(phases) != work:
                raise ValueError("Slow-cycle phases do not sum to work")
            records.append(dict(ordinal=ordinal, work_us=work/24, lateness_us=lateness/24,
                phases_us=dict(zip(NAMES[:stages], [v/24 for v in phases])),
                worker_us=dict(zip(NAMES[:stages], [v/24 for v in values[3+stages:]]))))
        result["slow_cycles"] = sorted(records, key=lambda r: r["work_us"], reverse=True)
    result["limits"] = "Elapsed wall time including instrumentation and IRQs; worker overlap is not general ISR accounting."
    return result

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    print(json.dumps(decode(args.directory), indent=2))
