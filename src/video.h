#ifndef VIDEO_H
#define VIDEO_H

#include <stdbool.h>

void run_frame();

// "Fit to calculator screen" mode. When true, the GBA frame is rendered
// to its native 240x160 region as usual, then scaled up by 7/5 to a
// 336x224 region centered horizontally on the 396x224 CG50 screen,
// just before dupdate. Aspect-correct; 30 px black bars on each side.
//
// gba_fit_init() must be called once at startup before toggling the
// flag -- it allocates a 240x160 source backup buffer used by the
// scaling pass. If allocation fails, fit mode silently stays off.
extern bool gba_fit_to_screen;
void gba_fit_init(void);

#endif