#ifndef SH4_JIT_H
#define SH4_JIT_H

#include <stdbool.h>

// SH4 JIT scaffolding. Phase 0 of docs/SH4_JIT_PLAN.md: prove that we
// can allocate a buffer, emit native SH4 code into it, and call it
// from C without the calculator crashing. Everything Phase 1+ depends
// on this returning cleanly.

// Result of the most recent jit_spike_run(). Inspected by main.c at
// boot and dprint'd onto the diagnostic screen. The status string is
// always populated, even on failure; jit_spike_ok flips to true only
// after a successful return from the emitted function.
extern bool jit_spike_ok;
extern char jit_spike_status[80];

// kmalloc a 32-byte buffer, emit `rts; nop`, icbi the line, and call
// it. The status string is written before the call so a crash leaves
// "calling buf=..." on screen for diagnosis. On a clean return we
// overwrite it with "ok".
void jit_spike_run(void);

#endif
