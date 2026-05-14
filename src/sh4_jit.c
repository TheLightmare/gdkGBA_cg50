#include "sh4_jit.h"

#include <stdio.h>
#include <stdint.h>

#include <gint/kmalloc.h>

bool jit_spike_ok = false;
char jit_spike_status[80] = "jit-spike: not run";

// Minimal SH4 routine: `rts; nop`.
//
// SH4 has a branch delay slot, so `rts` needs a following instruction
// that runs before the actual return takes effect. `nop` is the safe
// filler. Encodings match gint's own dynamic-code template at
// src/cpu/ics.s (which uses 0x000b for rts and 0x0009 for nop).
//
// Toolchain is sh3eb-elf (big-endian). A 16-bit aligned uint16_t
// store of 0x000b writes 0x00 then 0x0b -- the correct in-memory
// representation of the 16-bit instruction word.
#define SH4_RTS 0x000b
#define SH4_NOP 0x0009

void jit_spike_run(void) {
    // 32 bytes = one SH4 cache line. We only need 4 bytes for two
    // instructions, but a full-line allocation guarantees we don't
    // share the line with unrelated heap data and need only one icbi.
    uint16_t *code = kmalloc(32, "extram");
    if (!code) code = kmalloc(32, NULL);  // fall back to default arena
    if (!code) {
        snprintf(jit_spike_status, sizeof(jit_spike_status),
                 "jit-spike: kmalloc(32) failed");
        return;
    }

    code[0] = SH4_RTS;
    code[1] = SH4_NOP;

    // Invalidate the I-cache line holding our emitted code. Without
    // this, an instruction fetch at `code` could hit stale I-cache
    // data (whatever was there before the buffer was reused) instead
    // of reading the bytes we just wrote.
    //
    // gint's cpu_csleep machinery uses the same single-icbi pattern
    // for its 20-byte sleep template, so a 4-byte payload in a
    // line-aligned buffer is well within the proven recipe.
    __asm__ __volatile__("icbi @%0" :: "r"(code) : "memory");

    // Pre-populate the status string with the "about to call" state so
    // that if the call traps and the calculator panics, the diagnostic
    // screen still shows we got to this point.
    snprintf(jit_spike_status, sizeof(jit_spike_status),
             "jit-spike: calling buf=%p", code);

    typedef void (*native_fn)(void);
    ((native_fn)code)();

    // Reaching this line means the emitted code returned cleanly.
    jit_spike_ok = true;
    snprintf(jit_spike_status, sizeof(jit_spike_status),
             "jit-spike: ok (buf=%p)", code);
}
