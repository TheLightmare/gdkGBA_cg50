#include "save_file.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "arm_shared.h"

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
                // Mark EEPROM in use so rom_eep_read services reads from
                // the loaded buffer instead of falling through to ROM.
                eeprom_used = true;
                ok = true;
            }
            break;

        case SAVE_TYPE_FLASH:
            if (data_size == FLASH_BYTES) {
                // Flash is lazy-allocated by ensure_flash(); allocate it
                // up front here so the loaded data has somewhere to go.
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

    // Casio's filesystem ("wb" via gint stdio) tolerates re-creation of an
    // existing file, but remove() first keeps behaviour predictable across
    // OS revisions and lets fopen("wb") start from a clean slate.
    remove(path);

    FILE *f = fopen(path, "wb");
    if (!f) return false;

    uint8_t hdr[SAVE_HEADER_SIZE];
    memcpy(hdr, SAVE_MAGIC, 4);
    wr_u32_le(hdr +  4, SAVE_VERSION);
    wr_u32_le(hdr +  8, save_type);
    wr_u32_le(hdr + 12, data_size);

    bool ok = (fwrite(hdr,  1, sizeof(hdr), f) == sizeof(hdr))
           && (fwrite(data, 1, data_size,   f) == data_size);

    fclose(f);

    if (ok) save_dirty = false;
    return ok;
}
