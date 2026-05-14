#ifndef ARM_JIT_H
#define ARM_JIT_H

#include <stdbool.h>

#include "arm_block.h"
#include "sh4_jit.h"

// JIT compiler for ARM basic blocks. Mirrors thumb_jit.h: walks a
// freshly-decoded block's uop list and emits an SH4 routine that runs
// the same handlers via JSR. Phase 1 Chunk 4: framework only, no
// opcode specialisation yet -- every uop's existing C handler is
// invoked via JSR. The point is to validate the codegen end-to-end
// on the heavier (40 ms/frame) ARM cycle budget before any per-opcode
// inlining.
//
// On success: installs the resulting function pointer on
// b->native_entry and returns true. On failure (arena full, block too
// long for the literal-pool disp, fixup table overflow): the arena
// cursor is rewound to where it was on entry, native_entry is left
// untouched, and the caller's interpreter path keeps working.
bool arm_jit_compile_block(arm_block_t *b);

#endif
