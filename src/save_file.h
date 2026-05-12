#ifndef SAVE_FILE_H
#define SAVE_FILE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Save type the .sav file claims to hold. Determined at write time from
// the runtime flags eeprom_used / flash_used (see arm_mem.c). Encoded as
// a u32 in the on-disk header so future versions can grow the set.
typedef enum {
    SAVE_TYPE_NONE   = 0,
    SAVE_TYPE_SRAM   = 1,
    SAVE_TYPE_EEPROM = 2,
    SAVE_TYPE_FLASH  = 3,
} save_type_e;

// Set by save-region writes (eeprom_write / flash_write) and observed by
// the main loop, which debounces and flushes after a quiescent period.
extern bool save_dirty;

// Build a save-file path from a ROM path: a trailing ".gba" (case-insensitive)
// is replaced with ".sav"; otherwise ".sav" is appended. Returns false if the
// result would not fit in out_size.
bool save_make_path(const char *rom_path, char *out, size_t out_size);

// Try to load a previously written save file. Returns true if a valid file
// was found and applied (which also sets eeprom_used / flash_used so the
// emulator routes reads to the loaded buffer); false otherwise.
bool save_load(const char *path);

// Flush the currently in-use save buffer to disk. Picks the buffer based on
// flash_used / eeprom_used (see save_file.c). Clears save_dirty on success.
bool save_write(const char *path);

// Flag the in-use save buffer as having unsaved changes. Inlined into the
// hot write paths in arm_mem.c.
static inline void save_mark_dirty(void) { save_dirty = true; }

#endif
