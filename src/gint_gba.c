#include <gint/display.h>
#include "gint_gba.h"

void gint_gba_init(){
    // The GBA frame is rendered directly into gint_vram (the system-managed
    // 396x224 RGB565 framebuffer), so we do not need to allocate a separate
    // texture. Clear the framebuffer once so the unused margins around the
    // 240x160 GBA viewport stay black instead of showing earlier UI.
    dclear(C_BLACK);
}

void gint_gba_uninit(){
}
