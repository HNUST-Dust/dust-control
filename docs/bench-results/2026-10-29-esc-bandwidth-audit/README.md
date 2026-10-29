# ESC bandwidth audit — 2026-10-29

Scope: keep current PDO layout and A trace-off board image. Read-only board
register checks, actual saved ELF review, official SDK/manual review, and
reanalysis of existing trace-on samples. No clock/config write, flash, reset,
arbitrary PDRAM write, or new timing measurement was performed.

## Findings and limits

Current evidence favors a substantial hardware-access-path cost. It excludes
an accidental byte-at-a-time loop and per-word memcpy on the current400B SSC
path. It does NOT establish HPM's physical lower bound or quantify how much
of the remaining time is CPU loop, bridge wait, ESC arbitration, or preemption.
Do not label50us an unavoidable limit, or claim a measured bandwidth increase.

Current board readback: PDI_CTRL0140=0x80, PDI_CFG0150=0x81,
PDI_EXT0152=0.0000 (uint16). HPM manual section57.4.28 decodes BUS=100 as OPB
and CLK=1 as synchronous1*25MHz. This is what the configuration reports, not
an independent clock probe. PDI_CFG is read-only; no supported runtime PDI
multiplier adjustment was found. Section57.2.3 specifies a100MHz functional
clock, also used to derive the PHY25MHz reference. Increasing CPU frequency
or blindly changing ESC functional clock does not establish faster PDI access.

Saved board image abba-off-A: aPdInputData=0x0121c21c, 4B aligned. Destination
0x1d00 and400B length qualify for the word path. esc_write at0x800119d6 uses
100 aligned lw/sw iterations; inner loop0x80011a56..68 contains seven
instructions per word (branch/add/lw/add/addi/sw/jump), no memcpy/fence call.
There are memcpy calls in other branches, including unaligned buffers; their
presence in the function does not show that the SSC400B path executes them.
Trace-off wrapper is a short tail call to the API. Driver validation and API
calls are fixed costs; CPU loop and MMIO transaction waits are still present.
Instruction count alone cannot quantify stalls on this pipeline/cache/bus.

Official SDK HW_EscWrite uses ESCMEMCPY, scalar accessors use direct MMIO.
No reviewed official sample supplies a faster block transfer or a latency
benchmark. Support initialization enables clock_esc0; official board setup also
enables its group. No supported runtime speed-up was identified. The driver
comment about reset/preset100MHz is not an independent frequency measurement.

## Existing measurements, not new bench results

All are DTR1v7,64 samples per image, read96B and write400B. Intervals bracket
the SSC wrapper and can include trace lookup, driver work, IRQs and worker
preemption. Timer resolution1/24MHz=41.67ns. No independent clocks subtracted.

| Image | Read96 min/median/mean us | Write400 min/median/mean us | Write effective Mbit/s |
| --- | --- | --- | ---: |
| Baseline | 13.708 /13.708 /14.225 | 50.292 /51.292 /53.018 | 60.36 |
| Late | 13.750 /13.750 /14.285 | 48.250 /48.458 /49.439 | 64.73 |
| Fast | 13.708 /13.792 /14.467 | 48.250 /49.000 /49.755 | 64.32 |

Effective payload rate is400*8/time_us, not physical Ethernet throughput or
an isolated PDI clock measurement. One 32-bit word averages about494–530ns
on writes and593–603ns on reads, with unequal fixed overhead per word.
Minima are contaminated estimates too;64 samples do not establish a floor.
Write maxima71–81us show why mean alone cannot describe intrinsic bus latency.

Beckhoff Xilinx IP document Table58 gives OPB N=1 32-bit write360–440ns and
read440–560ns. Serialized arithmetic gives400B write36–44us and96B read
10.56–13.44us. Similar scale supports the hardware-access hypothesis, but this
is FPGA synthesis-dependent data, NOT an HPM5E31 spec. The difference from
48–53us cannot be assigned to software or treated as guaranteed recoverable
headroom. Exact HPM IP version/bridge timing is not established.

## Remaining decisive measurement

Preserve400B PDO and all field semantics. Instrument the actual normal SSC
word loop in a dedicated lab build, not unused or active PDRAM with synthetic
writes. Record chunk boundaries during that same full400B write (for example
16,64,128,192B contiguous chunks summing400) without changing address order,
width, first/last-byte access count or SyncManager commit behavior. Measure
read chunks separately within the normal96B read; do not compare different
lengths/directions as one fitted curve.

Use minimum instrumentation, bounded aggregate/histogram counters, timer-pair
calibration and an otherwise identical non-MMIO RAM loop control. Keep IRQs
and worker enabled. Sample many complete writes, report lower quantiles and
long tail separately; if no scheduler/ISR accounting is collected, do not
call any sample interruption-free. Compare two loop shapes (current versus
bounded unroll) under identical conditions; benefit from unroll estimates
reducible loop work, but is not a pure hardware/software decomposition.
Verify generated instructions, transfer span/order, matching ELF, and full
contracts before flash. Stop master before RAM export. No implementation or
new flash is part of this read-only audit.

A linear per-word slope with little loop-shape sensitivity would favor PDI/
bridge limits. A large RAM-control/fixed cost or meaningful unroll improvement
would justify software optimization. Even this measures CPU-visible effective
service time; true OPB cycle latency needs vendor confirmation or internal bus
observation. Ask HPM to confirm PDI clock, bridge latency, burst support and
supported timing configuration using the readbacks and disassembly above.

## Primary references

- HPM HPM5E00UM Rev0.1,57.2.3 and57.4.28:
  https://www.hpmicro.com/Public/Uploads/uploadfile/files/20250505/HPM5E00UMV01.pdf
- Beckhoff Xilinx IP2.04e,10.5/Table58 (FPGA comparison only):
  https://download.beckhoff.com/download/Document/io/ethercat-development-products/ethercat_ipcore_xilinx_v2_04e_datasheet_v1i0.pdf
- Official SDK samples/ethercat/port/hpm_ecat_hw.h:44–64;
  boards/hpm5e00evk/board.c board_init_ethercat.
- Support drivers/ethercat/esc_hpmicro.c:56–178,333–352.

Raw timing summaries, A-image disassembly and identity/readbacks saved beside
this report. Previous SDK review is in ../2026-10-29-esc-write-review/README.md.
