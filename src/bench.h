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

#else  // !GBA_BENCH

// Release builds: bench is fully compiled out. Provide an inline no-op
// bench_init() so the call in main() doesn't need to be #if'd. The other
// symbols are not declared on purpose -- any code that still references
// them in release would be a bug, and the link error is the right signal.
static inline void bench_init(void) {}

#endif

#endif
