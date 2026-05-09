#include "extram.h"

#include <stdio.h>
#include <string.h>

#include <gint/kmalloc.h>
#include <gint/hardware.h>

bool      extram_available = false;
uint32_t  extram_size      = 0;
char      extram_status[64] = "extram: not yet probed";

static kmalloc_arena_t extended_ram = { 0 };

// CG-50 OS version string lives at 0x80020020 ("03.06.0200" etc).
static bool os_version_supports_extram(void) {
    const char *osv = (const char *)0x80020020;

    // Expect "0M.NN.RRRR" where M is major, NN is minor.
    // Slyvtt's gate: major must be "03" and minor (the digit at osv[3])
    // must be '6' or less. OS 03.07 onwards repurposes 0x8c200000+.
    if (osv[0] != '0' || osv[1] != '3') return false;
    if (osv[2] != '.')                  return false;
    if (osv[3] > '6')                   return false;
    return true;
}

void extram_init(void) {
    if (extram_available) return;  // already done

    // Only fx-CG50 family is known good for the hardcoded address.
    if (gint[HWCALC] != HWCALC_FXCG50 &&
        gint[HWCALC] != HWCALC_FXCG_MANAGER) {
        snprintf(extram_status, sizeof(extram_status),
                 "extram: skipped (HWCALC=%d, not CG50)",
                 gint[HWCALC]);
        return;
    }

    if (!os_version_supports_extram()) {
        const char *osv = (const char *)0x80020020;
        snprintf(extram_status, sizeof(extram_status),
                 "extram: skipped (OS %c%c.%c%c too new)",
                 osv[0], osv[1], osv[3], osv[4]);
        return;
    }

    // For HWCALC_FXCG50 the region is 0x8c200000..0x8c4e0000.
    // For HWCALC_FXCG_MANAGER (the emulator) it's the P1 mirror at
    // 0x88200000..0x884e0000. Same physical memory, different alias.
    if (gint[HWCALC] == HWCALC_FXCG_MANAGER) {
        extended_ram.start = (void *)0x88200000;
        extended_ram.end   = (void *)0x884e0000;
    } else {
        extended_ram.start = (void *)0x8c200000;
        extended_ram.end   = (void *)0x8c4e0000;
    }

    extended_ram.name        = "extram";
    extended_ram.is_default  = false;  // explicit allocation only

    kmalloc_init_arena(&extended_ram, true);
    if (!kmalloc_add_arena(&extended_ram)) {
        snprintf(extram_status, sizeof(extram_status),
                 "extram: kmalloc_add_arena failed");
        return;
    }

    extram_available = true;
    extram_size = (uint32_t)((uint8_t *)extended_ram.end -
                             (uint8_t *)extended_ram.start);
    snprintf(extram_status, sizeof(extram_status),
             "extram: %u KB at %p available",
             (unsigned)(extram_size / 1024), extended_ram.start);
}
