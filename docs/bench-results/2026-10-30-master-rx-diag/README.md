# Isolated native igc RX timestamp diagnostic prepared

Status: BUILT + OFFLINE TESTED, NOT LOADED. Existing installed driver and
selected FreeRun B firmware remain unchanged; master Idle, slave PREOP, linkUP.
No live hardware timestamp or latency conclusion yet.

## Source / build evidence

User confirmed the old wbr_realtime_lab tree was deleted; no source/backup was
found in ~/dust-control. Recovered official source from
https://gitlab.com/etherlab.org/ethercat at stable-1.7 commit
e38a8787c3b0e842f566a2fe0485a5d0dd16ba0d. This does not recover the original
build commit/configuration. However, baseline driver's .text section is
byte-identical to the installed native module (SHA256 in manifest).
Built on NUC with its installed5.15.0-1114-realtime headers and GCC11.4.
Kernel vermagic matches. Both baseline and diagnostic driver are linked using
actual installed ec_master exports via KBUILD_EXTRA_SYMBOLS; every original
imported symbol CRC matches installed ec_igc. Added kernel imports are resolved
against the same running-kernel headers. No force-load or CRC bypass. We do
not replace/rebuild the running master. Early full-tree builds exposed different
master configuration CRCs; those master binaries are never used for deployment.

Artifacts on NUC: ~/dust-ecat-bench/rx-diag/{baseline,diagnostic,experiment}.
Original modules are additionally copied under rx-diag/original-ec_*.ko.
Existing /lib/modules files are never overwritten. See manifest for reviewed
original/master/candidate hashes and source commit. Patch and generator are
retained; the GPL diagnostic header is archived at `docs/handoff/2026-10-30-retired-experiments/igc-rx-timestamp.tar.gz` (member `tools/experiments/igc_rx_timestamp/rx_diag.h`).

## Diagnostic behavior

Load-only ecat_rx_diag:0=off,1=software records,2=hardware RX timestamps.
Hardware mode uses existing igc PTP helper with RX_FILTER_ALL, TX_OFF, only
for selected native PCI function0000:58:00.0. Fix native RX pointer/length to
skip the parsed16-byte hardware prefix. Reject corrupt short stamped descriptor
by ending that poll, rather than underflowing a size/passing malformed memory.
A resulting bad frame or WKC fault invalidates measurement; no retry policy.

Modes1/2 preallocate262144 records*64B (~16MiB), arm/stop through root debugfs.
No cyclic printk, allocation or full-frame copy. Metadata records hardware
PHC timestamp, host monotonic poll start and pre-ring-write record boundary,
first EtherCAT datagram index/address/length and validated optional Rx96/Tx400
LRW publication/accepted batch. This optional identity is not guessed for
split/reordered/non-LRW frames. Overflow, missing stamps, malformed records,
ambiguous sample joins fail the decoder. Reading while armed is rejected;
rearming while a reader is open is rejected. One selected BDF only; do not use
this prototype as a multi-device capture facility.

calibrate.py uses read-only bracketed clock_gettime(PHC) and CLOCK_MONOTONIC;
no clock adjustment. decode.py uses exact rational clock conversion before
reporting microseconds, to avoid large PHC epoch cancellation errors. Linear
drift/no-step assumption and bracket uncertainty are explicit. A negative
NIC-to-poll value can mean arrival during the poll, not corrupt data. The
record timestamp precedes ring write/ecdev_receive; it is not a hardware
arrival stamp or exact delivery-call entry.

Validation: kernel module builds without warnings; original .text equivalence;
vermagic and all original import CRCs match; decoder6 tests PASS (large epoch,
PHC conversion, arrival during poll, software-only mode, overflow/truncation/
missing stamps, ambiguous joins, clock step/order); Python compile and bash -n
PASS. Hardware disable-mode equivalence, ring runtime, timestamp behavior and
service restoration are not yet tested. No cyclic send/receive phase change.

## First controlled live test

Firmware B was freshly reset after restoring it. Stop leaves Core Protected;
every later independent session still needs gateway reset. The first command
runs diagnostics OFF,2000 CAN3 ID3-disable probes at125us, then restores original
EtherCAT service/driver on exit. This checks replacement + native path before
turning on hardware timestamps. Do not run a second session without reset.

sudo bash ~/dust-ecat-bench/rx-diag/experiment/probe.sh 0 master-rx-off-01

Wrapper verifies hashes, dedicated BDF, inactive master and original active
service before changes. It stops the service, unbinds only the dedicated BDF
from the standard igc that ethercatctl stop reloads, loads the installed master
with saved parameters and temporarily insmods the diagnostic module. It does
not unload unrelated NIC drivers. Uses existing benchmark/ASPM wrapper.
On exit unload candidate and restart original service; if master remains
active or unloading fails, never force unload and report the failure.
Logs: ~/dust-ecat-bench/rx-diag/results/master-rx-off-01/.
Later, after this check passes and gateway reset, software/hardware modes can
collect metadata plus all2000 RTT samples in one run with unchanged cadence.
