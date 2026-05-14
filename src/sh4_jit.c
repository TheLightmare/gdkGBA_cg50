#include "sh4_jit.h"

#include <stdio.h>
#include <stdint.h>

#include <gint/kmalloc.h>

#include "arm_block.h"
#include "bench.h"
#include "build_flags.h"
#include "thumb_block.h"

// ----- Arena ---------------------------------------------------------------

bool jit_enabled = false;
char jit_init_status[80] = "jit-arena: not initialised";

// 256 KB total arena, split into two 128 KB generations for partial
// eviction. When the active generation fills, we switch to the other
// one and clear only the native_entry pointers that fall into the
// soon-to-be-overwritten gen. Blocks recently JIT'd into the other
// gen survive the recycle and keep executing natively.
//
// SH4 L1 line is 32 bytes; the icbi loop in jit_emit_end walks one
// line at a time.
#define JIT_ARENA_BYTES (256u * 1024u)
#define JIT_GEN_BYTES   (JIT_ARENA_BYTES / 2u)
#define SH4_CACHE_LINE   32u

static uint16_t *jit_arena_base   = NULL;
static uint16_t *jit_arena_end    = NULL;
static uint16_t *jit_arena_cursor = NULL;

// Generations. jit_gens[active_gen] is where the cursor lives; the
// other gen holds the "previous" half-arena of JIT'd code, still valid
// and callable. Cursor is always within [jit_gens[active_gen].base,
// jit_gens[active_gen].end).
typedef struct {
    uint16_t *base;
    uint16_t *end;
} jit_gen_t;
static jit_gen_t jit_gens[2];
static int      jit_active_gen = 0;

bool jit_init(void) {
    if (jit_enabled) return true;

    void *p = kmalloc(JIT_ARENA_BYTES, "extram");
    if (!p) p = kmalloc(JIT_ARENA_BYTES, NULL);
    if (!p) {
        snprintf(jit_init_status, sizeof(jit_init_status),
                 "jit-arena: kmalloc(%uK) failed",
                 (unsigned)(JIT_ARENA_BYTES / 1024));
        return false;
    }

    jit_arena_base   = (uint16_t *)p;
    jit_arena_end    = (uint16_t *)((uint8_t *)p + JIT_ARENA_BYTES);
    jit_gens[0].base = jit_arena_base;
    jit_gens[0].end  = (uint16_t *)((uint8_t *)jit_arena_base + JIT_GEN_BYTES);
    jit_gens[1].base = jit_gens[0].end;
    jit_gens[1].end  = jit_arena_end;
    jit_active_gen   = 0;
    jit_arena_cursor = jit_gens[0].base;
    jit_enabled = true;

    snprintf(jit_init_status, sizeof(jit_init_status),
             "jit-arena: %uK @ %p",
             (unsigned)(JIT_ARENA_BYTES / 1024), (void *)jit_arena_base);
    return true;
}

void jit_reset(void) {
    if (!jit_enabled) return;
    jit_active_gen   = 0;
    jit_arena_cursor = jit_gens[0].base;
}

void jit_arena_recycle(void) {
    if (!jit_enabled) return;
    // Switch the active generation. The newly-active gen is about to
    // be overwritten, so clear native_entry on every block that points
    // into its range. Blocks that JIT'd into the OTHER gen (now the
    // inactive one) are untouched and keep executing natively until
    // the next recycle. This roughly halves the per-recycle re-JIT
    // pressure vs the chunk-9 full-clear policy.
    jit_active_gen ^= 1;
    jit_gen_t *now_active = &jit_gens[jit_active_gen];
    arm_block_clear_native_entries_in(now_active->base, now_active->end);
    thumb_block_clear_native_entries_in(now_active->base, now_active->end);
    jit_arena_cursor = now_active->base;
#if GBA_BENCH
    bench_jit_arena_recycles++;
#endif
}

uint16_t *jit_emit_begin(size_t budget_bytes) {
    if (!jit_enabled) return NULL;
    jit_gen_t *active = &jit_gens[jit_active_gen];
    size_t remaining = (size_t)((uint8_t *)active->end -
                                (uint8_t *)jit_arena_cursor);
    if (remaining < budget_bytes) {
        // Active generation exhausted: switch + clear the other gen +
        // retry. After switching the full generation is available; if
        // a single block exceeds even that, bail.
        jit_arena_recycle();
        active = &jit_gens[jit_active_gen];
        remaining = (size_t)((uint8_t *)active->end -
                             (uint8_t *)jit_arena_cursor);
        if (remaining < budget_bytes) return NULL;
    }
    return jit_arena_cursor;
}

void jit_emit16(uint16_t inst) {
    *jit_arena_cursor++ = inst;
}

uint16_t *jit_cursor(void) {
    return jit_arena_cursor;
}

void jit_rewind(uint16_t *target) {
    if (!jit_enabled) return;
    if (target < jit_arena_base || target > jit_arena_end) return;
    jit_arena_cursor = target;
}

native_block_fn_t jit_emit_end(uint16_t *start) {
    // icbi every cache line spanning [start, cursor). Round start down
    // and end up to line boundaries so any sub-line landing positions
    // still cover the modified range exactly. gint's cpu_csleep uses a
    // single icbi for its 20-byte template; this is the same recipe
    // generalised to arbitrary lengths.
    uintptr_t lo = (uintptr_t)start             &  ~(uintptr_t)(SH4_CACHE_LINE - 1);
    uintptr_t hi = ((uintptr_t)jit_arena_cursor +   (SH4_CACHE_LINE - 1))
                                                &  ~(uintptr_t)(SH4_CACHE_LINE - 1);
    for (uintptr_t p = lo; p < hi; p += SH4_CACHE_LINE) {
        __asm__ __volatile__("icbi @%0" :: "r"(p) : "memory");
    }
    return (native_block_fn_t)start;
}

// ----- Phase 0 spike -------------------------------------------------------

bool jit_spike_ok = false;
char jit_spike_status[80] = "jit-spike: not run";

// SH4 instruction encodings used by the spike. Same constants gint uses
// in its cpu_csleep template (src/cpu/ics.s in the gint tree).
#define SH4_RTS 0x000b
#define SH4_NOP 0x0009

void jit_spike_run(void) {
    // 32 bytes = one cache line. We only need 4 bytes for rts+nop, but
    // a line-aligned-ish reservation keeps icbi accounting trivial.
    uint16_t *code = jit_emit_begin(32);
    if (!code) {
        snprintf(jit_spike_status, sizeof(jit_spike_status),
                 "jit-spike: arena unavailable (skipped)");
        return;
    }

    jit_emit16(SH4_RTS);
    jit_emit16(SH4_NOP);
    native_block_fn_t fn = jit_emit_end(code);

    // Pre-populate status before the call so a crash still leaves
    // diagnostic info on the boot screen.
    snprintf(jit_spike_status, sizeof(jit_spike_status),
             "jit-spike: calling fn=%p", (void *)fn);

    // The spike fn does nothing with its argument -- the rts returns
    // immediately. Pass NULL for clarity.
    fn(NULL);

    jit_spike_ok = true;
    snprintf(jit_spike_status, sizeof(jit_spike_status),
             "jit-spike: ok (fn=%p)", (void *)fn);
}
