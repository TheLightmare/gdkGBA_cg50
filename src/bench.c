#include "bench.h"

#if GBA_BENCH

#include <gint/timer.h>
#include <gint/mpu/tmu.h>
#include <gint/clock.h>

uint64_t bench_arm_exec_ticks;
uint64_t bench_render_ticks;
uint64_t bench_dupdate_ticks;

uint64_t bench_arm_mode_ticks;
uint64_t bench_thumb_mode_ticks;

uint32_t bench_last_frame_us;

uint32_t bench_freq_hz;

uint32_t bench_mem_slow_read;
uint32_t bench_mem_slow_write;
uint32_t bench_chunk_miss;

uint32_t bench_arm_cond_skip;
uint32_t bench_arm_ss_inst;
uint32_t bench_arm_block_inst;
uint32_t bench_arm_block_decodes;
uint32_t bench_thumb_ss_inst;
uint32_t bench_thumb_block_inst;
uint32_t bench_thumb_block_decodes;

uint32_t bench_arm_legacy_inst;
uint32_t bench_arm_legacy_hist[16];

uint32_t bench_thumb_legacy_inst;
uint32_t bench_thumb_legacy_hist[16];

uint32_t bench_thumb_jit_compiled;
uint32_t bench_thumb_jit_specialized_ops;

uint32_t bench_arm_jit_compiled;
uint32_t bench_arm_jit_specialized_ops;

uint32_t bench_jit_arena_recycles;

static int bench_tmu_id = -1;
static uint32_t bench_tcor;

static int bench_callback_continue(void) {
    return TIMER_CONTINUE;
}

void bench_init(void) {
    // Request a standard TMU (0..2). ETMU's 32 kHz tick is too coarse for
    // sub-millisecond phase timing.
    //
    // 60-second delay → TCOR scales with whatever prescaler gint picks.
    // Long enough that no single bench_elapsed measurement can plausibly
    // span more than one wrap.
    bench_tmu_id = timer_configure(TIMER_TMU, 60000000ULL,
                                   GINT_CALL(bench_callback_continue));
    if (bench_tmu_id < 0 || bench_tmu_id > 2) {
        bench_tmu_id = -1;
        bench_freq_hz = 0;
        return;
    }

    // Capture the TCOR that gint wrote, so wrap-detection knows the period.
    bench_tcor = SH7305_TMU.TMU[bench_tmu_id].TCOR;

    // Read the actual prescaler gint chose (TCR.TPSC, low 3 bits) instead
    // of assuming Pphi/4. timer_configure() picks a prescaler internally
    // based on the requested duration; for a 60-sec delay it picks
    // Pphi/16, not Pphi/4. With our previous assumption of Pphi/4, we
    // were under-reporting time by 4× (i.e., reporting 16 fps when the
    // actual rate was 4 fps).
    uint32_t tcr_tpsc = SH7305_TMU.TMU[bench_tmu_id].TCR.word & 7;
    uint32_t divisor;
    switch (tcr_tpsc) {
        case 0: divisor =   4; break;
        case 1: divisor =  16; break;
        case 2: divisor =  64; break;
        case 3: divisor = 256; break;
        default: divisor = 4; break;
    }

    timer_start(bench_tmu_id);

    // Refresh gint's cached clock-frequency struct from the hardware
    // before reading it. clock_freq() returns a pointer to a static
    // struct that holds whatever values gint computed last; if the user
    // ran Ptune4 *outside* the add-in to raise Pphi before launching us,
    // gint's cached struct still holds the original (lower) Pphi and
    // bench_freq_hz would be too low — under-reporting wall time and
    // over-reporting fps by the actual Pphi-multiplier factor. The
    // cpg_compute_freq() call re-reads the CPG registers and updates
    // the struct so we get the *currently active* Pphi.
    cpg_compute_freq();
    const clock_frequency_t *cf = clock_freq();
    bench_freq_hz = (uint32_t)(cf->Pphi_f / divisor);
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

#endif // GBA_BENCH
