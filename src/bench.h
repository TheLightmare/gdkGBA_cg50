#ifndef BENCH_H
#define BENCH_H

#include <stdint.h>

// Lightweight wall-clock benchmarking using a free-running TMU.
//
// One TMU is reserved at boot at Pphi/4 (~4 MHz, ~250 ns per tick on stock
// CG-50). bench_now() reads the current TCNT value, which counts down from
// TCOR to 0 then reloads. bench_elapsed(start) returns ticks since `start`,
// handling at most one wrap (so single-call duration must be < TCOR period,
// which is set to 1 second — far longer than any single frame phase).
//
// Phase accumulators (ticks): summed across all calls within a frame. They
// are reset each time a diagnostic snapshot reads them, so each snapshot
// reports the per-frame totals for the most recent frame.

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

// Frequency of the bench timer in Hz (so callers can convert to µs).
// 0 if no TMU could be reserved.
extern uint32_t bench_freq_hz;

// Event counters. Plain uint32_t, no wraparound concern at the rates we
// expect within a single snapshot interval.
extern uint32_t bench_mem_slow_read;
extern uint32_t bench_mem_slow_write;
extern uint32_t bench_chunk_miss;

#endif
