#ifndef BENCH_H
#define BENCH_H

#include <stdint.h>
#include "build_flags.h"

// Lightweight wall-clock benchmarking using a free-running TMU.
//
// One TMU is reserved at boot (Pphi/16 on stock CG-50). bench_now() reads
// the current TCNT value, which counts down from TCOR to 0 then reloads.
// bench_elapsed(start) returns ticks since `start`, handling at most one
// wrap (so single-call duration must be < TCOR period, set to 60 s).
//
// Phase accumulators (ticks): summed across all calls within a frame. They
// are reset by the diagnostic snapshot reader, so each snapshot reports
// the per-frame totals for the most recent frame.
//
// Hot-path code should use BENCH_TIME_BLOCK / BENCH_INC from build_flags.h
// rather than calling bench_now/bench_elapsed directly -- those macros
// vanish entirely in release builds.

#if GBA_BENCH

void bench_init(void);

uint32_t bench_now(void);                     // current TCNT (counts down)
uint32_t bench_elapsed(uint32_t start);       // ticks since start

// Phase ticks. Accumulated across one frame. Cleared when consumed.
extern uint64_t bench_arm_exec_ticks;
extern uint64_t bench_render_ticks;
extern uint64_t bench_dupdate_ticks;

// arm_exec internal phase ticks: time spent in each mode's inner
// dispatch loop. Their sum should track bench_arm_exec_ticks closely
// (minus the outer-while + int_halt-check overhead).
extern uint64_t bench_arm_mode_ticks;
extern uint64_t bench_thumb_mode_ticks;

// Last frame's measured wall time (microseconds). Updated each frame in
// main.c so the heartbeat overlay can display "what the bench thinks the
// fps is" alongside the user's visual perception.
extern uint32_t bench_last_frame_us;

// Frequency of the bench timer in Hz (so callers can convert to us).
// 0 if no TMU could be reserved.
extern uint32_t bench_freq_hz;

// Event counters. Plain uint32_t, no wraparound concern at the rates we
// expect within a single snapshot interval.
extern uint32_t bench_mem_slow_read;
extern uint32_t bench_mem_slow_write;
extern uint32_t bench_chunk_miss;

// arm_exec instruction-mix counters. Both modes split block-cache vs.
// single-step paths so we can read coverage and decode traffic. Total
// instructions per mode = ss + block.
//   * ARM vs Thumb instruction split  -> (arm_ss+arm_blk) vs (thumb_*)
//   * Block-cache coverage            -> block_inst / (block_inst+ss_inst)
//   * Block-decode cold-path traffic  -> block_decodes per interval
//   * ARM condition-fail rate         -> cond_skip / (arm_ss+arm_blk)
extern uint32_t bench_arm_cond_skip;
extern uint32_t bench_arm_ss_inst;
extern uint32_t bench_arm_block_inst;
extern uint32_t bench_arm_block_decodes;
extern uint32_t bench_thumb_ss_inst;
extern uint32_t bench_thumb_block_inst;
extern uint32_t bench_thumb_block_decodes;

// Within the ARM block path, count how many uops fell back to the
// generic arm_dec_call_legacy handler (i.e. were NOT covered by a
// specialised arm_dec_* function), plus a 16-bucket histogram of
// those raw ops keyed on (raw_op >> 24) & 0xF. The histogram tells
// us which encoding families dominate the unspecialised work --
// directly actionable for picking the next handlers to specialise.
//
// Top-nibble bucketing (after the cond field):
//   0x0/0x1 = data-proc reg, MUL/halfword/signed mem, MSR-reg, BX
//   0x2/0x3 = data-proc imm, MSR-imm
//   0x4/0x5 = LDR/STR imm offset (various P/U/W combos)
//   0x6/0x7 = LDR/STR reg offset
//   0x8/0x9 = LDM/STM
//   0xA/0xB = B / BL
//   0xC/0xD = coprocessor data transfer
//   0xE     = CDP/MRC/MCR
//   0xF     = SWI
extern uint32_t bench_arm_legacy_inst;
extern uint32_t bench_arm_legacy_hist[16];

// Same coverage gauge for Thumb. There's no thumb-mode equivalent to
// arm_dec_call_legacy that we instrumented previously, so until this
// counter existed we had no visibility into the Thumb block-cache
// coverage. Bucketed on (raw_op >> 12) & 0xF -- bits 15..12 of the
// 16-bit Thumb opcode, which cleanly partition the encoding tree:
//   0x0 = LSL/LSR/ASR Rd, Rm, #imm5 ; ADD/SUB reg or imm3
//   0x1 = ditto (top5 continuation)
//   0x2 = MOV/CMP Rd, #imm8
//   0x3 = ADD/SUB Rd, #imm8
//   0x4 = data-proc reg, special data, BX/BLX reg, LDR PC-rel
//   0x5 = LDR/STR/STRH/LDRH/LDRSB/LDRSH (register offset)
//   0x6 = LDR/STR Rd, [Rn, #imm5*4]
//   0x7 = LDRB/STRB Rd, [Rn, #imm5]
//   0x8 = LDRH/STRH Rd, [Rn, #imm5*2]
//   0x9 = LDR/STR Rd, [SP, #imm8*4]
//   0xA = ADD Rd, PC/SP, #imm8*4
//   0xB = misc (PUSH/POP, ADD/SUB SP, etc.)
//   0xC = LDM/STM
//   0xD = conditional B / SWI
//   0xE = unconditional B / BLX prefix
//   0xF = BL prefix/suffix
extern uint32_t bench_thumb_legacy_inst;
extern uint32_t bench_thumb_legacy_hist[16];

#else  // !GBA_BENCH

// Release builds: bench is fully compiled out. Provide an inline no-op
// bench_init() so the call in main() doesn't need to be #if'd. The other
// symbols are not declared on purpose -- any code that still references
// them in release would be a bug, and the link error is the right signal.
static inline void bench_init(void) {}

#endif

#endif
