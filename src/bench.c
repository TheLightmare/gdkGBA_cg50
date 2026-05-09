#include "bench.h"

#include <gint/timer.h>
#include <gint/mpu/tmu.h>
#include <gint/clock.h>

uint64_t bench_arm_exec_ticks;
uint64_t bench_render_ticks;
uint64_t bench_dupdate_ticks;

uint32_t bench_freq_hz;

uint32_t bench_mem_slow_read;
uint32_t bench_mem_slow_write;
uint32_t bench_chunk_miss;

static int bench_tmu_id = -1;
static uint32_t bench_tcor;

static int bench_callback_continue(void) {
    return TIMER_CONTINUE;
}

void bench_init(void) {
    // Request a standard TMU (0..2). ETMU's 32 kHz tick is too coarse for
    // sub-millisecond phase timing.
    //
    // 60-second delay → TCOR ≈ 60 × Pphi/4. Long enough that no single
    // bench_elapsed measurement can plausibly span more than one wrap, so
    // the wrap-detection branch in bench_elapsed handles it correctly.
    // (Pphi/4 prescaler can count up to ~350 sec per gint docs.)
    bench_tmu_id = timer_configure(TIMER_TMU, 60000000ULL,
                                   GINT_CALL(bench_callback_continue));
    if (bench_tmu_id < 0 || bench_tmu_id > 2) {
        bench_tmu_id = -1;
        bench_freq_hz = 0;
        return;
    }

    // Capture the TCOR that gint wrote, so wrap-detection knows the period.
    bench_tcor = SH7305_TMU.TMU[bench_tmu_id].TCOR;

    timer_start(bench_tmu_id);

    // Pphi at default Pphi/4 prescaler is the bench tick clock. clock_freq()
    // returns the active clock-pulse-generator state.
    const clock_frequency_t *cf = clock_freq();
    bench_freq_hz = (uint32_t)(cf->Pphi_f / 4);
}

uint32_t bench_now(void) {
    if (bench_tmu_id < 0) return 0;
    return SH7305_TMU.TMU[bench_tmu_id].TCNT;
}

uint32_t bench_elapsed(uint32_t start) {
    if (bench_tmu_id < 0) return 0;
    uint32_t now = SH7305_TMU.TMU[bench_tmu_id].TCNT;
    // TCNT counts down from TCOR to 0, then reloads to TCOR. Two cases:
    //   - no wrap: now <= start, elapsed = start - now
    //   - one wrap: TCNT went S → 0 → TCOR → N, elapsed = S + (TCOR - N)
    // The previous version assumed unsigned subtract handled the wrap; it
    // doesn't (TCOR ≠ 2^32), so wrapped measurements got credited ~2^32
    // ticks (~580 s on the original 1-sec TCOR). 60-sec TCOR + this branch
    // gives correct values as long as no single measurement exceeds 60 s.
    if (now <= start) {
        return start - now;
    }
    return start + (bench_tcor - now);
}
