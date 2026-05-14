#ifndef SH4_JIT_H
#define SH4_JIT_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

// SH4 JIT scaffolding for fxgba. See docs/SH4_JIT_PLAN.md for the full
// phased rollout. This header exposes (1) the arena + emit primitives
// Phase 1 codegen will build on top of, and (2) the native-block entry
// type that arm_block_t/thumb_block_t carry so the executor can branch
// between "walk uops" and "call native code".
//
// Phase 0 deliverable ("always-fail" JIT): every primitive below is
// implemented and the block-cache wiring is in place, but the decoders
// never install a non-NULL native_entry. The executor's native branch
// is therefore unreachable -- it exists only to verify the layout
// change compiles and links cleanly.

// Native block entry. The SH4 first-argument register (R4) carries a
// pointer to arm_r (passed as void * to avoid pulling arm.h into every
// header that mentions the type). The function returns when the block
// has finished -- branch taken, IRQ window, or block-length cap. The
// executor pre-credits b->total_cycles into arm_cycles before the
// call, so native code does NOT need to do its own cycle accounting
// in Phase 1; Phase 3 will revisit this with lazy/bulk accounting.
typedef void (*native_block_fn_t)(void *register_file);

// ----- Arena ---------------------------------------------------------------

// jit_enabled flips to true after a successful jit_init(). When false,
// jit_emit_begin() always returns NULL so callers naturally fall back
// to the interpreter path.
extern bool jit_enabled;
extern char jit_init_status[80];

// Allocate the code arena (currently 256 KB) from the extram kmalloc
// pool, falling back to the default arena. Idempotent. Returns true
// iff jit_enabled is set on exit. Safe to call before any block-cache
// decoding has happened.
bool jit_init(void);

// Reset the bump cursor to the arena base. All previously emitted
// code is invalidated; callers MUST clear native_entry on every block
// that pointed into the arena before calling this. Phase 1+ only;
// Phase 0 never invokes it.
void jit_reset(void);

// Begin a new block. Returns the current cursor (= entry point for
// the block being emitted) if at least budget_bytes remain in the
// arena; NULL otherwise.
uint16_t *jit_emit_begin(size_t budget_bytes);

// Emit one 16-bit SH4 instruction at the current cursor. Caller is
// responsible for honouring the budget passed to jit_emit_begin.
void jit_emit16(uint16_t inst);

// Finalize the block: icbi every cache line spanning [start, cursor).
// Returns the entry pointer cast to native_block_fn_t so the caller
// can install it on a block_t. `start` is the value jit_emit_begin
// returned.
native_block_fn_t jit_emit_end(uint16_t *start);

// ----- Phase 0 spike -------------------------------------------------------

// Round-trips through jit_emit_begin/16/end and calls the result. On
// clean return, jit_spike_ok is true and jit_spike_status reads "ok".
// Independent of jit_enabled in the sense that a disabled arena
// reports "skipped" rather than crashing.
extern bool jit_spike_ok;
extern char jit_spike_status[80];
void jit_spike_run(void);

#endif
