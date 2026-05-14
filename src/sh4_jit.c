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

// 256 KB initial budget per docs/SH4_JIT_PLAN.md. Phase 1+ can raise this
// if frequent jit_reset() calls indicate eviction pressure. SH4 L1 line
// is 32 bytes; the icbi loop in jit_emit_end walks one line at a time.
#define JIT_ARENA_BYTES (256u * 1024u)
#define SH4_CACHE_LINE   32u

static uint16_t *jit_arena_base   = NULL;
static uint16_t *jit_arena_end    = NULL;
static uint16_t *jit_arena_cursor = NULL;

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
    jit_arena_cursor = jit_arena_base;
    jit_enabled = true;

    snprintf(jit_init_status, sizeof(jit_init_status),
             "jit-arena: %uK @ %p",
             (unsigned)(JIT_ARENA_BYTES / 1024), (void *)jit_arena_base);
    return true;
}

void jit_reset(void) {
    if (!jit_enabled) return;
    jit_arena_cursor = jit_arena_base;
}

void jit_arena_recycle(void) {
    if (!jit_enabled) return;
    // Clear native_entry on every directory slot before recycling the
    // arena. Old pointers would refer to about-to-be-overwritten
    // memory, so any future executor lookup that hit them would jump
    // into stale or partially-rewritten code. NULL is the universal
    // "fall back to interpreter" signal.
    arm_block_clear_native_entries();
    thumb_block_clear_native_entries();
    jit_arena_cursor = jit_arena_base;
#if GBA_BENCH
    bench_jit_arena_recycles++;
#endif
}

uint16_t *jit_emit_begin(size_t budget_bytes) {
    if (!jit_enabled) return NULL;
    size_t remaining = (size_t)((uint8_t *)jit_arena_end -
                                (uint8_t *)jit_arena_cursor);
    if (remaining < budget_bytes) {
        // Arena exhausted: recycle and retry. After recycling, the full
        // arena is available; if even that can't fit the request the
        // block is genuinely too big and we bail.
        jit_arena_recycle();
        remaining = (size_t)((uint8_t *)jit_arena_end -
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
