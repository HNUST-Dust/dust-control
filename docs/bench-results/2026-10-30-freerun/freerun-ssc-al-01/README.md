# FreeRun SSC configuration audit / unused AL-event read experiment

Current comparison baseline freerun-return-01: CAN3-only125us/2000 PASS,
RTTmean755.877us,p95873.797us,max999.005us,zero timeout/WKC miss.

Compiler -dM/-E with the actual ecatappl compile command (see effective-config.json):
AL_EVENT_ENABLED1, INTERRUPTS_SUPPORTED1, DC_SUPPORTED0, ECAT_TIMER_INT0;
ESC_16BIT_ACCESS0, ESC_32BIT_ACCESS0. PDRAM word access is independently enabled
in the HPM driver; those SSC alignment flags do not make the current400B transfer
bytewise. Do not blindly force32bit register access with side-effect registers.
MIN_PD_CYCLE_TIME is overridden to1000000ns by gateway_config.h, not the500000ns
shown in the imported tool config. It describes application capability, not
the125us FreeRun host frame scheduler. Timing metadata needs a separate accurate
production review; do not lower declarations to claim faster execution.
PD_INPUT/OUTPUT_CALC_AND_COPY_TIME both0: auto measurement code remains.
MAX input1536/output1024 are capacities, not active Tx400/Rx96 packet lengths.
Do not shrink maxima expecting current packets to shrink.

Actual baseline disassembly shows PDO_InputMapping calls hpm5e_ssc_al_event
unconditionally when the auto timing branch is compiled. Its local event value
is never used. This experiment removes ONLY that read/swap pair in the recognized
build copy of the function; original licensed external/ssc is never rewritten.
Optional CONFIG_DUST_ECAT_SSC_SKIP_UNUSED_AL_READ defaultn, FreeRun-only.
No fake nonzero calc-copy constants; timing measurement and SDO values remain.
Reject source variants if the event has any remaining use or pattern differs.
Actual-function native ASan/UBSan tests compare baseline/adjusted mapping writes
and timer state for measurement off/on and all static/auto timing combinations;
PASS. Target build PASS ROM174988/RAM211024; disassembly confirms the one call
is gone. No host scheduling/PDO/identity/watchdog change; no extra profiling.

NUC: sudo bash ~/dust-ecat-bench/ethercat_rtt_aspm_probe.sh 125 2000 freerun-ssc-al-01

Preserve original normal receive/process/send order, no fixed receive offset.
Saved matching ELF/HEX/config under firmware build/bench-host/freerun-ssc-al-01.
Live validation pending. A single run cannot establish a small RTT gain.

Primary SSC reference: https://download.beckhoff.com/download/document/io/ethercat-development-products/an_et9300_v1i10.pdf
Next master-side question: distinguish wire/frame completion from application
observation with timestamps. Historical receive mean0.875us and final wake
mean0.059us do not demonstrate NIC/driver slowness or a recoverable125us.

Live run complete PASS2000, no timeout/WKC miss; post-stop bDcSyncActive0 and
bEscIntEnabled0. RTTmean718.904us,p50748.514,p95873.319,max883.710us.
Previous baseline mean755.877,p50748.660,p95873.797,max999.005us.
Mean change -36.973us is NOT the execution time saved by one register read.
Nearest125us band populations: baseline625/750/875/1000us =255/1388/343/14;
experiment625/750/875us =607/1260/133. Replication remains required.

Existing sample timestamps show send call mean0.547us; receive entry through
application consume mean2.167us. These exclude sleeping before receive and
unknown NIC residence; neither proves driver/hardware arrival timing.
Accepted-observed timestamps are host observations, not actual MCU accept.
Do not infer command-vs-feedback execution split from these timestamps.

Keep current image experimental, default flagn. Correction: each independent bench session requires a gateway reset/new boot
ID after Stop. The previously suggested loop without reset failed in02 with
zero samples;03 was not run. Preserve protection and reset the same image
between independent sessions before inspecting within-image band variation.
All dumps performed after master exit, without reset, then CPU resumed.
