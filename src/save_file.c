#include "save_file.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <gint/gint.h>

#include "arm_shared.h"

// save_write runs DURING gameplay (the debounced auto-save). On a fragmented
// .sav file, an unwrapped fopen+fwrite from gint world can panic AND corrupt
// the file -- the "saves got wiped" symptom. Same rationale as rom_buffer's
// chunk-load path: the Fugue filesystem's fragment-table lookup needs
// OS-world state, so the I/O must run inside a gint_world_switch.
//
// save_load is intentionally NOT world-switched here. It runs once at boot,
// before the per-frame loop ever starts; the OS world is in a clean state
// and bare stdio works. Wrapping it during early boot caused freezes (likely
// some gint subsystem isn't fully ready for nested switches at that point).
struct save_write_args {
    const char    *path;
    const uint8_t *hdr;
    size_t         hdr_size;
    const uint8_t *data;
    size_t         data_size;
    bool           ok;
};

static int do_save_write(struct save_write_args *a) {
    remove(a->path);
    FILE *f = fopen(a->path, "wb");
    if (!f) { a->ok = false; return -1; }
    a->ok = (fwrite(a->hdr,  1, a->hdr_size,  f) == a->hdr_size)
         && (fwrite(a->data, 1, a->data_size, f) == a->data_size);
    fclose(f);
    return 0;
}

// Set by arm_mem.c on the first save-region write of each kind. We do not
// own these; we just observe them at write time to decide which buffer to
// dump, and assert them at load time so subsequent reads route correctly.
extern bool eeprom_used;
extern bool flash_used;

bool save_dirty = false;

#define SAVE_MAGIC      "GSAV"
#define SAVE_VERSION    1u

// Buffer sizes mirror the allocations in arm_init() / ensure_flash().
#define SRAM_BYTES      0x10000u  // 64 KB
#define EEPROM_BYTES    0x2000u   //  8 KB (covers both 4Kbit and 64Kbit chips)
#define FLASH_BYTES     0x20000u  // 128 KB (two 64 KB banks)

// On-disk header: 16 bytes, all little-endian. The SH4A on the CG50 is
// big-endian, so we serialize fields by hand instead of writing the struct
// directly — keeps .sav files swappable with desktop tools and other ports.
#define SAVE_HEADER_SIZE 16

static uint32_t rd_u32_le(const uint8_t *p) {
    return  (uint32_t)p[0]
         | ((uint32_t)p[1] <<  8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static void wr_u32_le(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >>  0);
    p[1] = (uint8_t)(v >>  8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

bool save_make_path(const char *rom_path, char *out, size_t out_size) {
    size_t len = strlen(rom_path);
    const char *dot = strrchr(rom_path, '.');

    bool has_gba_ext =
        dot &&
        (dot[1] == 'g' || dot[1] == 'G') &&
        (dot[2] == 'b' || dot[2] == 'B') &&
        (dot[3] == 'a' || dot[3] == 'A') &&
         dot[4] == '\0';

    size_t base_len = has_gba_ext ? (size_t)(dot - rom_path) : len;
    if (base_len + 4 + 1 > out_size) return false;

    memcpy(out, rom_path, base_len);
    memcpy(out + base_len, ".sav", 4);
    out[base_len + 4] = '\0';
    return true;
}

bool save_load(const char *path) {
    // Unwrapped fopen/fread/fclose. save_load runs once at startup, before
    // the emulator's per-frame loop and the chunk-loading world switches,
    // so the OS is in a clean state where bare stdio works even on lightly
    // fragmented files. The freeze we saw when this was world-switched
    // suggests recursive/early gint_world_switch calls aren't safe at this
    // boot stage. save_write, which runs DURING gameplay, still goes
    // through the world-switched path -- that's where save corruption
    // actually originates.
    FILE *f = fopen(path, "rb");
    if (!f) return false;

    uint8_t hdr[SAVE_HEADER_SIZE];
    if (fread(hdr, 1, sizeof(hdr), f) != sizeof(hdr)) {
        fclose(f);
        return false;
    }
    if (memcmp(hdr, SAVE_MAGIC, 4) != 0) {
        fclose(f);
        return false;
    }

    uint32_t version   = rd_u32_le(hdr +  4);
    uint32_t save_type = rd_u32_le(hdr +  8);
    uint32_t data_size = rd_u32_le(hdr + 12);
    if (version != SAVE_VERSION) {
        fclose(f);
        return false;
    }

    bool ok = false;
    switch (save_type) {
        case SAVE_TYPE_SRAM:
            if (data_size == SRAM_BYTES && sram &&
                fread(sram, 1, SRAM_BYTES, f) == SRAM_BYTES) {
                ok = true;
            }
            break;

        case SAVE_TYPE_EEPROM:
            if (data_size == EEPROM_BYTES && eeprom &&
                fread(eeprom, 1, EEPROM_BYTES, f) == EEPROM_BYTES) {
                eeprom_used = true;
                ok = true;
            }
            break;

        case SAVE_TYPE_FLASH:
            if (data_size == FLASH_BYTES) {
                if (!flash) {
                    flash = malloc(FLASH_BYTES);
                    if (!flash) break;
                    for (uint32_t i = 0; i < FLASH_BYTES; i++) flash[i] = 0xff;
                }
                if (fread(flash, 1, FLASH_BYTES, f) == FLASH_BYTES) {
                    flash_used = true;
                    ok = true;
                }
            }
            break;

        default:
            break;
    }

    fclose(f);
    return ok;
}

bool save_write(const char *path) {
    // Pick the buffer the game has actually been touching. flash takes
    // precedence: flash games also write command bytes through the SRAM
    // array, so an early SRAM-looking write isn't conclusive.
    uint32_t      save_type;
    const uint8_t *data;
    uint32_t      data_size;

    if (flash_used && flash) {
        save_type = SAVE_TYPE_FLASH;
        data      = flash;
        data_size = FLASH_BYTES;
    } else if (eeprom_used && eeprom) {
        save_type = SAVE_TYPE_EEPROM;
        data      = eeprom;
        data_size = EEPROM_BYTES;
    } else if (sram) {
        save_type = SAVE_TYPE_SRAM;
        data      = sram;
        data_size = SRAM_BYTES;
    } else {
        return false;
    }

    uint8_t hdr[SAVE_HEADER_SIZE];
    memcpy(hdr, SAVE_MAGIC, 4);
    wr_u32_le(hdr +  4, SAVE_VERSION);
    wr_u32_le(hdr +  8, save_type);
    wr_u32_le(hdr + 12, data_size);

    // World-switched truncate + fopen("wb") + fwrite hdr + fwrite data +
    // fclose. The whole sequence runs in OS world so the fragment-table
    // lookup is valid.
    struct save_write_args args = {
        .path = path,
        .hdr  = hdr,  .hdr_size  = sizeof(hdr),
        .data = data, .data_size = data_size,
        .ok   = false,
    };
    gint_world_switch(GINT_CALL(do_save_write, (void *)&args));

    if (args.ok) save_dirty = false;
    return args.ok;
}
