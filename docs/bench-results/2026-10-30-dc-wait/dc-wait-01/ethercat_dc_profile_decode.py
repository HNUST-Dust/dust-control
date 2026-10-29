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

def decode_budget(directory):
    b = (directory / "dust_sync0_budget_stats.bin").read_bytes()
    if len(b) != 56:
        raise ValueError("Unexpected budget RAM size")
    version, count, total, maximum, late = struct.unpack_from("<IIQII", b)
    bins = struct.unpack_from("<7I", b, 24)
    if version != 1 or not count or sum(bins) != count:
        raise ValueError("Inconsistent budget histogram")
    return dict(kind="budget", version=version, services=count,
        mean_us=total/count/24, max_us=maximum/24, max_lateness_us=late/24,
        histogram_us=dict(zip(["lt100", "100to125", "125to150", "150to200",
                              "200to250", "250to500", "ge500"], bins)),
        limits="Elapsed Sync0 handler time, excludes PDI/outer loop; no per-phase hooks.")

WAIT_NAMES=["Other","SemaphoreWait","PDI","Sync0Owner","PollBeforeSSC",
            "SSCMainLoop","PollAfterSSC","LinkMaintenance","InactivePHY","Idle"]
def decode_wait(directory):
    raw=(directory/"dust_sync0_wait_stats.bin").read_bytes()
    if len(raw)!=536: raise ValueError("Unexpected wait profile size")
    h=struct.unpack_from("<8I",raw)
    version,count,phases,max_late,max_ack,max_wake,max_checkpoint,kept=h
    totals=struct.unpack_from("<6Q",raw,32)
    owners=struct.unpack_from("<10Q",raw,80)
    origins=struct.unpack_from("<10I",raw,160)
    histogram=struct.unpack_from("<7I",raw,200)
    if version!=1 or not count or phases!=10 or kept>4 or sum(origins)!=count or sum(histogram)!=count:
        raise ValueError("Inconsistent wait profile counters")
    records=[]
    for i in range(kept):
        r=struct.unpack_from("<19I",raw,232+76*i)
        if sum(r[9:])+r[6]!=r[2]: raise ValueError("Wait phase attribution does not sum to latency")
        records.append(dict(ordinal=r[0],origin=WAIT_NAMES[r[1]],late_us=r[2]/24,
            irq_ack_us=r[3]/24,wake_call_us=r[4]/24,first_checkpoint_us=r[5]/24,
            detach_to_start_us=r[6]/24,worker_overlap_us=r[7]/24,irq_overlap_us=r[8]/24,
            owner_us=dict(zip(WAIT_NAMES,[v/24 for v in r[9:]]))))
    return dict(services=count,mean_late_us=totals[0]/count/24,max_late_us=max_late/24,
        mean_ack_us=totals[1]/count/24,max_ack_us=max_ack/24,
        mean_wake_call_us=totals[2]/count/24,max_wake_call_us=max_wake/24,
        mean_first_checkpoint_us=totals[3]/count/24,max_first_checkpoint_us=max_checkpoint/24,
        mean_worker_overlap_us=totals[4]/count/24,mean_irq_overlap_us=totals[5]/count/24,
        owner_mean_us=dict(zip(WAIT_NAMES,[v/count/24 for v in owners])),
        notification_origins=dict(zip(WAIT_NAMES,origins)),
        histogram_us=dict(zip(["lt10","10to25","25to50","50to100","100to200","200to250","ge250"],histogram)),
        slow_waits=sorted(records,key=lambda r:r["late_us"],reverse=True),
        limits="Owner checkpoints are not kernel ready/switch events; IRQ and worker overlap are not additive. Notification occurs after IRQ acknowledgement.")

def decode(directory):
    if not (directory / "dust_sync0_phase_stats.bin").exists():
        result=decode_budget(directory)
        if (directory / "dust_sync0_wait_stats.bin").exists(): result["wait"]=decode_wait(directory)
        return result
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
