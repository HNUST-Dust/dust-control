# ESC write path / official SDK review — 2026-10-29

Read-only investigation; no firmware, clock, PDI configuration or board changes.

## Actual image and driver

Reviewed saved ssc-v7-01 ELF. `aPdInputData` is 0x01224e84, size 0x600,
and `aPdOutputData` 0x01225484. Both are 4-byte aligned in this image.
400-byte input writes at 0x1d00 satisfy the current PDRAM word path checks.
The actual aligned esc_write loop at 0x80012388 uses one aligned lw and one
volatile MMIO sw per four bytes, plus loop/address instructions. No per-word
memcpy call occurs in that branch. The unaligned branch does call memcpy,
but it is not the branch used by this image's SSC input buffer. This is an
ELF-specific finding; later layouts must be checked again. Config uses -Os,
I/D caches enabled. See saved disassembly and hashes.

## SDK comparison

User-specified official SDK paths reviewed:
- samples/ethercat/port/hpm_ecat_hw.h, lines 44–64: block writes call ESCMEMCPY,
  scalar byte/word/dword accessors directly dereference MMIO.
- samples/ethercat/port/hpm_ecat_hw.c, lines 506–526: explicit four-byte read
  helper for variants that require aligned word reads. It is not a DMA writer.
- samples/ethercat/ecat_io/ecat.c: sample executes MainLoop continuously.
- samples/ethercat/ecat_io/README_zh.rst: SSC generation/EEPROM/FreeRun/DC setup;
  no 400-byte write latency benchmark found in the reviewed material.
- boards/hpm5e00evk/board.c, board_init_ethercat: clock group enable, pin/PHY
  setup. No runtime PDI frequency increase is provided by this function.

Current imported SSC ecat_def.h defaults ESCMEMCPY to memcpy. A cast to UINT32*
in a memcpy macro does not itself guarantee a particular transaction width.
Current explicit volatile word loop provides exact width/order/span semantics.
The official helper alone is not evidence that replacing this loop is faster.
Current support driver's historical native/board word-access results also show
~49 us writes after removing per-word memcpy, so this is not a new alignment fix.

## Read-only hardware evidence

Via NUC `ethercat reg_read -p 0` after benchmark completion:
- 0x0140 uint8 = 0x80 (on-chip bus)
- 0x0150 uint8 = 0x81
- 0x0152 uint16 = 0x0000

HPM5E00 UM Rev0.1 section 57.4.28 describes PDI_CFG BUS[7:5]=100 as OPB and
CLK[4:0]=1 as synchronous 1*25 MHz. These fields are read-only. Thus the
register reports an OPB PDI with factor 1, distinct from section 57.2.3's
100 MHz ESC functional clock. This is register interpretation, not independent
physical clock measurement. Do not write PDI_CFG or change PLL/ESC clocks on
this evidence; no supported runtime tuning mechanism has been established.

## Related Beckhoff IP timing: hypothesis, not HPM specification

Beckhoff IP Core for Xilinx v2.04e, section 10.5.4/table 58, gives OPB at N=1:
32-bit write 360–440 ns; read 440–560 ns. Arithmetic for serialized accesses:
100 writes / 400 B: 36–44 us; 24 reads / 96 B: 10.56–13.44 us. These are
similar in scale to existing HPM measurements (~50 us write, ~14 us read).
The FPGA document notes synthesis dependence; HPM's exact integration / IP
version / bridge behavior has not been established from it. Therefore this
supports a hardware-access-dominated hypothesis, not a proven lower bound or
an HPM performance guarantee. It also does not establish DMA burst support.

Primary sources:
- HPM UM: https://www.hpmicro.com/Public/Uploads/uploadfile/files/20250505/HPM5E00UMV01.pdf
  sections 57.2.3 and 57.4.28 (PDF pages 1098, 1127–1128).
- Beckhoff: https://download.beckhoff.com/download/Document/io/ethercat-development-products/ethercat_ipcore_xilinx_v2_04e_datasheet_v1i0.pdf
  section 10.5, table 58, printed III-106 / PDF page index117.
ETG ESC overview URL returned 403, so it was not used for new conclusions.

## Next useful test

Measure multiple valid PDO lengths through the real SSC path with CPU word
access, unchanged bus conditions, and account for IRQ/worker time inside write.
Fit time = fixed overhead + word count * incremental cost; use minimally
instrumented measurements. Do not run arbitrary writes in an active SM buffer
or disable interrupts for a 400 B transfer merely to improve a number.
A large linear per-word component would favor reducing transmitted bytes or
redundant image writes; unrolling/O3 cannot eliminate the PDI transaction
latency. Repeated-write suppression requires separate SSC/SM validation, not
blindly replacing mapping with an early return. Keep synchronous DMA disabled.
