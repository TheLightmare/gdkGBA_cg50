# gdkGBA_cg50
port of gdkChan's gba emulator to fx-cg50

## The Goal

The goal is to port the emulator to a functional state. The second aim will be to optimize it as much as possible, to see if we can attain a playable state in a future implementation.

This precise implementation will most likely NOT be a good base for a playable emulator on the fx-cg50, because of the unoptimized nature of gdkGBA that was not made to run on a calculator to begin with.

## Current State

The project compiles using fxSDK and Gint and produces `fxgba.g3a`.

Recent fixes (untested on hardware — calculator USB cable still pending):

- The chunk-loading ROM buffer (`rom_buffer.c`) is now actually built — it had been written but never added to `CMakeLists.txt`.
- `screen` is now wired to `texture->data`, so the renderer no longer writes through a NULL pointer.
- The framebuffer was switched from SDL-style 32-bit BGRA to native Gint RGB565 (palette in `arm_mem.c`, all `*(uint32_t*)screen` writes in `video.c`). This also halves the framebuffer footprint (75 KB instead of 150 KB).
- `rom_buffer_read_16/32` now resolve the chunk once per call instead of doing a full LRU lookup per byte.
- Input was rewritten to use `keydown()` polling so directions register hold/release correctly. Buttons mapped: arrows (D-pad), SHIFT (A), ALPHA (B), F1 (L), F6 (R), OPTN (Select), EXE (Start), MENU/EXIT (quit).
- Dead `sdl.c`/`sdl.h` removed.

Known remaining issues: `sound.c` still uses `double` extensively (slow on SH4, no FPU benefit), audio is not piped to any output, no ROM picker (still hardcoded `test.gba`), and the GBA core itself is unoptimized for SH4.
