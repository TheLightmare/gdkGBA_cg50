#include "sh4_jit.h"

#include <stdio.h>
#include <stdint.h>

#include <gint/kmalloc.h>

// ----- Arena ---------------------------------------------------------------

bool jit_enabled = false;
char jit_init_status[80] = "jit-arena: not initialised";

// Phase 2 chunk 2: grow arena from 256 KB to 1 MB. The Phase 1 256 KB
// budget filled in the first few frames (snap 0+1 of the chunk-1 trace
// showed >500 blocks compiled by frame 5, well past 256 KB at the
// conservative ~500 B/block estimate). Once full, jit_emit_begin
// returned NULL on every subsequent decode -- so chunk 1's inline
// data-proc REG specs applied only to ~20 ARM boot blocks and never
// reached the gameplay hot path. Growing the arena targets that
// coverage gap directly: extram has plenty of headroom on the CG-50
// (~4 MB usable after OS overhead) and the snap-6 working set is
// only ~1200 unique blocks (~600 KB at the conservative estimate), so
// 1 MB should hold it without thrash. SH4 L1 line is 32 bytes; the
// icbi loop in jit_emit_end walks one line at a time.
#define JIT_ARENA_BYTES (1024u * 1024u)
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

uint16_t *jit_emit_begin(size_t budget_bytes) {
    if (!jit_enabled) return NULL;
    size_t remaining = (size_t)((uint8_t *)jit_arena_end -
                                (uint8_t *)jit_arena_cursor);
    if (remaining < budget_bytes) return NULL;
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
