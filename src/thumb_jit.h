#ifndef THUMB_JIT_H
#define THUMB_JIT_H

#include <stdbool.h>

#include "thumb_block.h"
#include "sh4_jit.h"

// JIT compiler for Thumb basic blocks. Walks a freshly-decoded block's
// uop list and emits an SH4 routine that runs the same handlers via
// JSR. Phase 1 Chunk 1: no opcode specialization yet -- every uop's
// existing handler is invoked via JSR. The point of this chunk is to
// validate the codegen framework end-to-end before any per-opcode
// inlining. See docs/SH4_JIT_PLAN.md.
//
// On success: installs the resulting function pointer on
// b->native_entry and returns true. On failure (arena full, block too
// long for the literal pool's 8-bit disp, fixup table overflow): the
// arena cursor is rewound to where it was on entry, native_entry is
// left untouched, and the caller's interpreter path keeps working.
bool thumb_jit_compile_block(thumb_block_t *b);

#endif
