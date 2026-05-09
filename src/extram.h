#ifndef EXTRAM_H
#define EXTRAM_H

#include <stdbool.h>
#include <stdint.h>

// Optional "extram" arena at 0x8c200000..0x8c4e0000 (3 MB).
//
// On the fx-CG50 with OS version 03.06 or older, this region is unused
// physical RAM that gint doesn't manage by default. Slyvtt's Shmup project
// established that we can register it as a kmalloc arena (named "extram")
// and allocate from it freely. Newer OS versions ( >= 03.07 ) reserve the
// region for themselves; using it on those would crash the calculator, so
// the registration is gated on an OS-version check.
//
// Unlike _ostk (which is the OS stack and gets clobbered by world-switches),
// extram is unrelated to OS execution -- it's safe across gint_world_switch.

extern bool      extram_available;
extern uint32_t  extram_size;        // bytes if available, 0 otherwise
extern char      extram_status[64];  // diagnostic string for the boot log

// Probe the OS version, register the extram arena if compatible. Should be
// called once, very early in main() (before any kmalloc that might benefit
// from the extra space).
void extram_init(void);

#endif
