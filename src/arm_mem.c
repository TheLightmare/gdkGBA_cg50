#include <stdlib.h>

#include <gint/defs/attributes.h>
#include "arm.h"
#include "arm_mem.h"
#include "bench.h"
#include "build_flags.h"
#include "io.h"
#include "mem_swizzle.h"
#include "rom_buffer.h"
#include "save_file.h"
#include "thumb_block.h"
#include "extram.h"

#include <gint/kmalloc.h>

#define EEPROM_WRITE  2
#define EEPROM_READ   3

// EXTERN VARIABLES DECLARATION =====

uint8_t bios[16384];

// 64 KB clamped EWRAM fallback when the extram arena is unavailable. The
// real GBA has 256 KB of EWRAM; if extram is available we allocate the
// full size from there and ignore this static buffer.
static uint8_t wram_board_static[0x10000];
uint8_t *wram_board = wram_board_static;
uint32_t ewram_size = 0x10000;
uint32_t ewram_mask = 0xFFFF;

static void arm_mem_pages_init(void);

void ewram_init(void) {
    if (extram_available) {
        uint8_t *p = (uint8_t *)kmalloc(0x40000, "extram");
        if (p) {
            wram_board = p;
            ewram_size = 0x40000;
            ewram_mask = 0x3FFFF;
            arm_mem_pages_init();
            return;
        }
    }
    // Fallback: keep the static 64 KB buffer with mirroring mask.
    wram_board = wram_board_static;
    ewram_size = 0x10000;
    ewram_mask = 0xFFFF;
    arm_mem_pages_init();
}

uint8_t wram_chip[0x00008000];
uint8_t palette_ram[0x00000400];
uint8_t vram[0x00018000];
// OAM stays in main RAM. We tried GXRAM placement; it gave no measurable
// improvement and may have slightly hurt (XYRAM 16-bit access has a small
// penalty vs cached main RAM, and 1 KB OAM was already cache-resident).
uint8_t oam[0x00000400];
// rom_buffer's chunk-cache state (last_chunk_address, last_chunk_idx, the
// 32 chunk_addresses[], the chunk_buffers[] pointer array, etc.) is read
// on every instruction fetch. Putting the struct in XYRAM means those
// reads cost 0 bus transactions; the actual chunk DATA still lives in
// main-RAM (the chunk buffers point to extram allocations, ~64 KB each).
//
// Struct size is ~308 bytes (FILE*, sizes, 32×ptr, 32×addr, 32×lru, etc.)
// which fits comfortably in the remaining XYRAM budget.
RomBuffer rom_buffer GXRAM;

// Page tables placed in XYRAM: hit on every memory operation, and main RAM
// reads cost a bus transaction (~117 MHz) while XYRAM reads run at CPU clock
// (1 cycle, never evicted). Bus traffic dominates this emulator's runtime
// since the user's diagnostic showed Ptune3 (CPU-only OC) does nothing while
// Ptune4 F5 (CPU + bus OC) helps measurably.
mem_page_t mem_read_pages[256] GXRAM;
static mem_page_t mem_write_pages[256] GXRAM;

// Populate mem_read_pages and mem_write_pages.
//
// EWRAM (0x02) and IWRAM (0x03) are deliberately NOT entered into the
// page table. Their backing buffers are stored in host-native byte order
// (see mem_swizzle.h) so the read/write fast paths can issue a single
// SH4 bus transaction instead of byte-by-byte assembly. The fast paths
// branch on (region == 0x02 / 0x03) explicitly before consulting the
// table.
//
// READ entries left in the table: PRAM (0x05) and OAM (0x07) -- both
// stored in raw guest byte order because their consumers (palette[]
// cache and the renderer's direct oam[] reads) expect that layout. VRAM
// (0x06) has a mirroring rule too complex for a single AND mask and is
// handled inline.
//
// WRITE entries left: only OAM (0x07). PRAM has a side effect (BGR555 ->
// RGB565 cache update) that needs the inlined fast path in arm_writeh /
// arm_write. VRAM is excluded because of the mirroring rule and the
// byte-write-duplicates-to-halfword quirk. Note OAM byte writes are
// silently dropped on real hardware; arm_writeb checks for OAM
// (region 7) before consulting the page table.
//
// Called from ewram_init() because wram_board / ewram_mask are only
// finalised there. The wram_board / ewram_mask init path no longer
// touches the page table for region 0x02 since that's handled by the
// explicit fast path in arm_readb / arm_readh / arm_read instead.
static void arm_mem_pages_init(void) {
    for (int i = 0; i < 256; i++) {
        mem_read_pages[i].base = NULL;
        mem_read_pages[i].mask = 0;
        mem_write_pages[i].base = NULL;
        mem_write_pages[i].mask = 0;
    }
    mem_read_pages[0x05].base = palette_ram;
    mem_read_pages[0x05].mask = 0x3FF;
    mem_read_pages[0x07].base = oam;
    mem_read_pages[0x07].mask = 0x3FF;

    mem_write_pages[0x07].base = oam;
    mem_write_pages[0x07].mask = 0x3FF;
}
uint8_t *eeprom;
uint8_t *sram;
uint8_t *flash;

// RGB565 palette cache: read by render_line on every visible pixel. We
// tried GXRAM placement — no measurable improvement, since 1 KB fits
// trivially in the 16 KB OC and was already cache-resident. Stays in
// main RAM.
uint16_t palette[0x200];

uint32_t bios_op;

uint32_t cart_rom_size;
uint32_t cart_rom_mask;

uint16_t eeprom_idx;

//======================

uint32_t flash_bank = 0;

typedef enum {
    IDLE,
    ERASE,
    WRITE,
    BANK_SWITCH
} flash_mode_e;

flash_mode_e flash_mode = IDLE;

bool flash_id_mode = false;

bool flash_used = false;

// Lazily allocate the 128 KB flash buffer the first time the game touches it.
// Returns true on success, false if the system is out of memory.
static bool ensure_flash(void) {
    if (flash) return true;
    flash = malloc(0x20000);
    if (!flash) return false;
    // GBA flash defaults to all-1s after a chip erase.
    for (uint32_t i = 0; i < 0x20000; i++) flash[i] = 0xff;
    return true;
}

bool eeprom_used = false;
bool eeprom_read = false;

uint32_t eeprom_addr      = 0;
uint32_t eeprom_addr_read = 0;

uint8_t eeprom_buff[0x100];

static const uint8_t bus_size_lut[16]  = { 4, 4, 2, 4, 4, 2, 2, 4, 2, 2, 2, 2, 2, 2, 1, 1 };

static void arm_access(uint32_t address, access_type_e at) {
    uint8_t cycles = 1;

    if (address & 0x08000000) {
        if (at == NON_SEQ)
            cycles += ws_n[(address >> 25) & 3];
        else
            cycles += ws_s[(address >> 25) & 3];
    } else if ((address >> 24) == 2) {
        cycles += 2;
    }

    arm_cycles += cycles;
}

static void arm_access_bus(uint32_t address, uint8_t size, access_type_e at) {
    uint8_t lut_idx = (address >> 24) & 0xf;
    uint8_t bus_sz = bus_size_lut[lut_idx];

    if (bus_sz < size) {
        arm_access(address + 0, at);
        arm_access(address + 2, SEQUENTIAL);
    } else {
        arm_access(address, at);
    }
}

//Memory read
static uint8_t bios_read(uint32_t address) {
    if ((address | arm_r.r[15]) < 0x4000)
        return bios[address & 0x3fff];
    else
        return bios_op;
}

// EWRAM mask is set at boot by ewram_init() to either 0x3FFFF (full
// 256 KB allocated from extram) or 0xFFFF (64 KB static fallback with
// mirroring of the upper region).

static uint8_t wram_read(uint32_t address) {
    return mem_swz_read_b(wram_board, address & ewram_mask);
}

static uint8_t iwram_read(uint32_t address) {
    return mem_swz_read_b(wram_chip, address & 0x7fff);
}

static uint8_t pram_read(uint32_t address) {
    return palette_ram[address & 0x3ff];
}

static uint8_t vram_read(uint32_t address) {
    return vram[address & (address & 0x10000 ? 0x17fff : 0x1ffff)];
}

static uint8_t oam_read(uint32_t address) {
    return oam[address & 0x3ff];
}

static uint8_t rom_read(uint32_t address) {
    return rom_buffer_read_8(&rom_buffer, address);
}

static uint8_t rom_eep_read(uint32_t address, uint8_t offset) {
    if (eeprom_used &&
        ((cart_rom_size >  0x1000000 && (address >>  8) == 0x0dffff) ||
         (cart_rom_size <= 0x1000000 && (address >> 24) == 0x00000d))) {
         if (!offset) {
             uint8_t mode = eeprom_buff[0] >> 6;

             switch (mode) {
                 case EEPROM_WRITE: return 1;
                 case EEPROM_READ: {
                    uint8_t value = 0;

                    if (eeprom_idx >= 4) {
                        uint8_t idx = ((eeprom_idx - 4) >> 3) & 7;
                        uint8_t bit = ((eeprom_idx - 4) >> 0) & 7;

                        value = (eeprom[eeprom_addr_read | idx] >> (bit ^ 7)) & 1;
                    }

                    eeprom_idx++;

                    return value;
                 }
             }
         }
    } else {
        return rom_read(address);
    }

    return 0;
}

static uint8_t flash_read(uint32_t address) {
    if (flash_id_mode) {
        //This is the Flash ROM ID, we return Sanyo ID code
        switch (address) {
            case 0x0e000000: return 0x62;
            case 0x0e000001: return 0x13;
        }
    } else if (flash_used) {
        if (!ensure_flash()) return 0xff;
        return flash[flash_bank | (address & 0xffff)];
    } else {
        return sram[address & 0xffff];
    }

    return 0;
}

static uint8_t arm_read_(uint32_t address, uint8_t offset) {
    switch (address >> 24) {
        case 0x0: return bios_read(address);
        case 0x2: return wram_read(address);
        case 0x3: return iwram_read(address);
        case 0x4: return io_read(address);
        case 0x5: return pram_read(address);
        case 0x6: return vram_read(address);
        case 0x7: return oam_read(address);

        case 0x8:
        case 0x9:
            return rom_read(address);

        case 0xa:
        case 0xb:
            return rom_read(address);

        case 0xc:
        case 0xd:
            return rom_eep_read(address, offset);

        case 0xe:
        case 0xf:
            return flash_read(address);
    }

    return 0;
}

#define IS_OPEN_BUS(a)  (((a) >> 28) || ((a) >= 0x00004000 && (a) < 0x02000000))

// Slow paths: the original switch-based implementation. Called only when
// the page-table fast path can't serve the read (BIOS/IO/VRAM/ROM/Flash/
// EEPROM, plus any unmapped region where open-bus behavior matters).
uint8_t arm_readb_slow(uint32_t address) {
    BENCH_INC(bench_mem_slow_read);
    uint8_t value = arm_read_(address, 0);

    if (!(address & 0x08000000)) {
        io_open_bus &= ((address >> 24) == 4);

        if (IS_OPEN_BUS(address) || io_open_bus)
            value = arm_pipe[1];
    }

    return value;
}

uint32_t arm_readh_slow(uint32_t address) {
    BENCH_INC(bench_mem_slow_read);
    uint32_t a = address & ~1;
    uint8_t  s = address &  1;
    uint32_t value;

    // Fast path for plain ROM reads (regions 0x8/0x9/0xa/0xb): one chunk
    // lookup instead of two byte-fetches that each lookup the chunk.
    uint8_t region = (a >> 24) & 0xf;
    if (region >= 0x8 && region <= 0xb) {
        value = rom_buffer_read_16(&rom_buffer, a);
    } else {
        value =
            arm_read_(a | 0, 0) << 0 |
            arm_read_(a | 1, 1) << 8;
    }

    if (!(a & 0x08000000)) {
        io_open_bus &= ((a >> 24) == 4);

        if (a < 0x4000 && arm_r.r[15] >= 0x4000)
            value = bios_op     & 0xffff;
        else if (IS_OPEN_BUS(a) || io_open_bus)
            value = arm_pipe[1] & 0xffff;
    }

    return ROR(value, s << 3);
}

uint32_t arm_read_slow(uint32_t address) {
    BENCH_INC(bench_mem_slow_read);
    uint32_t a = address & ~3;
    uint8_t  s = address &  3;
    uint32_t value;

    uint8_t region = (a >> 24) & 0xf;
    if (region >= 0x8 && region <= 0xb) {
        value = rom_buffer_read_32(&rom_buffer, a);
    } else {
        value =
            arm_read_(a | 0, 0) <<  0 |
            arm_read_(a | 1, 1) <<  8 |
            arm_read_(a | 2, 2) << 16 |
            arm_read_(a | 3, 3) << 24;
    }

    if (!(a & 0x08000000)) {
        io_open_bus &= ((a >> 24) == 4);

        if (a < 0x4000 && arm_r.r[15] >= 0x4000)
            value = bios_op;
        else if (IS_OPEN_BUS(a) || io_open_bus)
            value = arm_pipe[1];
    }

    return ROR(value, s << 3);
}

// Fast paths: explicit EWRAM/IWRAM branches using native loads on the
// host-swizzled buffers (one SH4 bus transaction per aligned access),
// then a page-table lookup for PRAM/OAM, plus an inlined VRAM branch.
// All fast paths have none of the open-bus / BIOS-leak post-processing
// because those regions are guaranteed-real-memory: IS_OPEN_BUS is
// false, the address has no 0x08000000 bit, and we're not in BIOS. The
// only side effect we need to preserve is `io_open_bus = 0`.
//
// VRAM (0x06) stays inlined rather than tabled because its mirroring
// rule (96 KB visible window, with the 32 KB high half mirrored at
// 0x18000) can't be expressed as a single AND mask.
uint8_t arm_readb(uint32_t address) {
    uint32_t reg = (address >> 24) & 0xFF;
    if (reg == 0x03) {
        io_open_bus = false;
        return mem_swz_read_b(wram_chip, address & 0x7FFF);
    }
    if (reg == 0x02) {
        io_open_bus = false;
        return mem_swz_read_b(wram_board, address & ewram_mask);
    }
    mem_page_t *p = &mem_read_pages[reg];
    if (p->base) {
        io_open_bus = false;
        return p->base[address & p->mask];
    }
    if (reg == 0x06) {
        uint32_t off = address & 0x1FFFF;
        if (off & 0x10000) off &= 0x17FFF;
        io_open_bus = false;
        return vram[off];
    }
    return arm_readb_slow(address);
}

uint32_t arm_readh(uint32_t address) {
    uint32_t a = address & ~1;
    uint32_t reg = (a >> 24) & 0xFF;
    if (reg == 0x03) {
        io_open_bus = false;
        uint32_t value = mem_swz_read_h(wram_chip, a & 0x7FFF);
        return ROR(value, (address & 1) << 3);
    }
    if (reg == 0x02) {
        io_open_bus = false;
        uint32_t value = mem_swz_read_h(wram_board, a & ewram_mask);
        return ROR(value, (address & 1) << 3);
    }
    mem_page_t *p = &mem_read_pages[reg];
    if (p->base) {
        io_open_bus = false;
        const uint8_t *src = p->base + (a & p->mask);
        uint32_t value = (uint32_t)src[0]
                       | ((uint32_t)src[1] << 8);
        return ROR(value, (address & 1) << 3);
    }
    if (reg == 0x06) {
        uint32_t off = a & 0x1FFFF;
        if (off & 0x10000) off &= 0x17FFF;
        io_open_bus = false;
        const uint8_t *src = vram + off;
        uint32_t value = (uint32_t)src[0]
                       | ((uint32_t)src[1] << 8);
        return ROR(value, (address & 1) << 3);
    }
    return arm_readh_slow(address);
}

uint32_t arm_read(uint32_t address) {
    uint32_t a = address & ~3;
    uint32_t reg = (a >> 24) & 0xFF;
    if (reg == 0x03) {
        io_open_bus = false;
        uint32_t value = mem_swz_read_w(wram_chip, a & 0x7FFF);
        return ROR(value, (address & 3) << 3);
    }
    if (reg == 0x02) {
        io_open_bus = false;
        uint32_t value = mem_swz_read_w(wram_board, a & ewram_mask);
        return ROR(value, (address & 3) << 3);
    }
    mem_page_t *p = &mem_read_pages[reg];
    if (p->base) {
        io_open_bus = false;
        const uint8_t *src = p->base + (a & p->mask);
        uint32_t value = (uint32_t)src[0]
                       | ((uint32_t)src[1] <<  8)
                       | ((uint32_t)src[2] << 16)
                       | ((uint32_t)src[3] << 24);
        return ROR(value, (address & 3) << 3);
    }
    if (reg == 0x06) {
        uint32_t off = a & 0x1FFFF;
        if (off & 0x10000) off &= 0x17FFF;
        io_open_bus = false;
        const uint8_t *src = vram + off;
        uint32_t value = (uint32_t)src[0]
                       | ((uint32_t)src[1] <<  8)
                       | ((uint32_t)src[2] << 16)
                       | ((uint32_t)src[3] << 24);
        return ROR(value, (address & 3) << 3);
    }
    return arm_read_slow(address);
}

uint8_t arm_readb_n(uint32_t address) {
    arm_access_bus(address, ARM_BYTE_SZ, NON_SEQ);

    return arm_readb(address);
}

uint32_t arm_readh_n(uint32_t address) {
    arm_access_bus(address, ARM_HWORD_SZ, NON_SEQ);

    return arm_readh(address);
}

uint32_t arm_read_n(uint32_t address) {
    arm_access_bus(address, ARM_WORD_SZ, NON_SEQ);

    return arm_read(address);
}

uint8_t arm_readb_s(uint32_t address) {
    arm_access_bus(address, ARM_BYTE_SZ, SEQUENTIAL);

    return arm_readb(address);
}

uint32_t arm_readh_s(uint32_t address) {
    arm_access_bus(address, ARM_HWORD_SZ, SEQUENTIAL);

    return arm_readh(address);
}

uint32_t arm_read_s(uint32_t address) {
    arm_access_bus(address, ARM_WORD_SZ, SEQUENTIAL);

    return arm_read(address);
}

//Memory write
static void wram_write(uint32_t address, uint8_t value) {
    mem_swz_write_b(wram_board, address & ewram_mask, value);
}

static void iwram_write(uint32_t address, uint8_t value) {
    mem_swz_write_b(wram_chip, address & 0x7fff, value);
}

static void pram_write(uint32_t address, uint8_t value) {
    palette_ram[address & 0x3ff] = value;

    address &= 0x3fe;

    uint16_t pixel = palette_ram[address] | (palette_ram[address + 1] << 8);

    uint16_t r = (pixel >>  0) & 0x1f;
    uint16_t g = (pixel >>  5) & 0x1f;
    uint16_t b = (pixel >> 10) & 0x1f;

    // GBA BGR555 -> RGB565: swap R/B; expand G from 5 to 6 bits.
    palette[address >> 1] = (r << 11) | (((g << 1) | (g >> 4)) << 5) | b;
}

static void vram_write(uint32_t address, uint8_t value) {
    vram[address & (address & 0x10000 ? 0x17fff : 0x1ffff)] = value;
}

static void oam_write(uint32_t address, uint8_t value) {
    oam[address & 0x3ff] = value;
}

static void eeprom_write(uint32_t address, uint8_t offset, uint8_t value) {
    if (!offset &&
    ((cart_rom_size >  0x1000000 && (address >>  8) == 0x0dffff) ||
     (cart_rom_size <= 0x1000000 && (address >> 24) == 0x00000d))) {
        if (eeprom_idx == 0) {
            //First write, erase buffer
            eeprom_read = false;

            uint16_t i;

            for (i = 0; i < 0x100; i++)
                eeprom_buff[i] = 0;
        }

        uint8_t idx = (eeprom_idx >> 3) & 0xff;
        uint8_t bit = (eeprom_idx >> 0) & 0x7;

        eeprom_buff[idx] |= (value & 1) << (bit ^ 7);

        if (++eeprom_idx == dma_ch[3].count.w) {
            //Last write, process buffer
            uint8_t mode = eeprom_buff[0] >> 6;

            //Value is only valid if bit 1 is set (2 or 3, READ = 2)
            if (mode & EEPROM_READ) {
                bool eep_512b = (eeprom_idx == 2 + 6 + (mode == EEPROM_WRITE ? 64 : 0) + 1);

                if (eep_512b)
                    eeprom_addr =   eeprom_buff[0] & 0x3f;
                else
                    eeprom_addr = ((eeprom_buff[0] & 0x3f) << 8) | eeprom_buff[1];

                eeprom_addr <<= 3;

                if (mode == EEPROM_WRITE) {
                    //Perform write to actual EEPROM buffer
                    uint8_t buff_addr = eep_512b ? 1 : 2;

                    // The original code did `uint64_t v = *(uint64_t*)(eeprom_buff + buff_addr)`
                    // -- buff_addr is 1 or 2 so the source is misaligned, and on SH4
                    // that triggers an alignment fault. Both source and destination
                    // are byte arrays; copy 8 bytes directly.
                    if (eeprom_addr + 8 <= 0x2000) {
                        for (int i = 0; i < 8; i++)
                            eeprom[eeprom_addr + i] = eeprom_buff[buff_addr + i];
                    }
                } else {
                    eeprom_addr_read = eeprom_addr;
                }

                eeprom_idx = 0;
            }
        }

        eeprom_used = true;
        save_mark_dirty();
    }
}

static void flash_write(uint32_t address, uint8_t value) {
    if (flash_mode == WRITE) {
        if (ensure_flash())
            flash[flash_bank | (address & 0xffff)] = value;

        flash_mode = IDLE;
    } else if (flash_mode == BANK_SWITCH && address == 0x0e000000) {
        flash_bank = (value & 1) << 16;

        flash_mode = IDLE;
    } else if (sram[0x5555] == 0xaa && sram[0x2aaa] == 0x55) {
        if (address == 0x0e005555) {
            //Command to do something on Flash ROM
            switch (value) {
                //Erase all
                case 0x10:
                    if (flash_mode == ERASE) {
                        if (ensure_flash()) {
                            uint32_t idx;

                            for (idx = 0; idx < 0x20000; idx++) {
                                flash[idx] = 0xff;
                            }
                        }

                        flash_mode = IDLE;
                    }
                break;

                case 0x80: flash_mode    = ERASE;       break;
                case 0x90: flash_id_mode = true;        break;
                case 0xa0: flash_mode    = WRITE;       break;
                case 0xb0: flash_mode    = BANK_SWITCH; break;
                case 0xf0: flash_id_mode = false;       break;
            }

            //We try to guess that the game uses flash if it
            //writes the specific flash commands to the save memory region
            if (flash_mode || flash_id_mode) {
                flash_used = true;
            }
        } else if (flash_mode == ERASE && value == 0x30) {
            if (ensure_flash()) {
                uint32_t bank_s = address & 0xf000;
                uint32_t bank_e = bank_s + 0x1000;
                uint32_t idx;

                for (idx = bank_s; idx < bank_e; idx++) {
                    flash[flash_bank | idx] = 0xff;
                }
            }

            flash_mode = IDLE;
        }
    }

    sram[address & 0xffff] = value;
    // Any write into the 0x0E.. region is either real SRAM data or a flash
    // command/data byte; in both cases the active save buffer is now stale.
    // Debouncing in main.c keeps the SRAM-mirrored flash command bursts
    // (0xAA/0x55 sequences) from triggering a flush per byte.
    save_mark_dirty();
}

static void arm_write_(uint32_t address, uint8_t offset, uint8_t value) {
    BENCH_INC(bench_mem_slow_write);
    switch (address >> 24) {
        case 0x2: wram_write(address, value); break;
        case 0x3: iwram_write(address, value); break;
        case 0x4: io_write(address, value); break;
        case 0x5: pram_write(address, value); break;
        case 0x6: vram_write(address, value); break;
        case 0x7: oam_write(address, value); break;

        case 0xc:
        case 0xd:
            eeprom_write(address, offset, value); break;

        case 0xe:
        case 0xf:
            flash_write(address, value); break;
    }
}

void arm_writeb(uint32_t address, uint8_t value) {
    uint8_t ah = address >> 24;

    // OAM (region 7) silently drops 8-bit writes on real hardware.
    if (ah == 7) return;

    // Fast path: EWRAM/IWRAM byte writes go to the host-swizzled buffers.
    // Phase 4c: also bump the page-gen counter so any cached Thumb block
    // covering this page is detected as stale on the next lookup.
    if (ah == 0x03) {
        mem_swz_write_b(wram_chip, address & 0x7FFF, value);
        thumb_block_invalidate_page(address);
        return;
    }
    if (ah == 0x02) {
        mem_swz_write_b(wram_board, address & ewram_mask, value);
        thumb_block_invalidate_page(address);
        return;
    }
    // Other page-table regions (only OAM remains writable through here,
    // and that's already handled above).
    mem_page_t *p = &mem_write_pages[ah];
    if (p->base) {
        p->base[address & p->mask] = value;
        return;
    }

    // OBJ VRAM also drops 8-bit writes: in modes 0-2 the OBJ tile area
    // starts at 0x06010000; in modes 3-5 it starts at 0x06014000.
    // Conservative test below covers both: any byte-write past the BG
    // VRAM area in mode-0..2 is dropped. (Mode 3-5 split is at 0x14000;
    // current_mode is small and rarely toggled at runtime so this
    // approximation works for typical games.)
    if (ah == 6) {
        uint32_t mode = disp_cnt.w & 7;
        uint32_t obj_base = (mode >= 3) ? 0x14000 : 0x10000;
        if ((address & 0x1ffff) >= obj_base) return;
    }

    // BG VRAM and palette duplicate the byte to a halfword (real GBA
    // behaviour for those regions).
    if (ah > 4 && ah < 8) {
        arm_write_(address + 0, 0, value);
        arm_write_(address + 1, 1, value);
    } else {
        arm_write_(address, 0, value);
    }
}

void arm_writeh(uint32_t address, uint16_t value) {
    uint32_t a = address & ~1;
    uint32_t ah = (a >> 24) & 0xFF;

    // Fast path: EWRAM/IWRAM halfword writes go to the host-swizzled
    // buffers as a single 16-bit store. Phase 4c: invalidate any cached
    // Thumb block covering this page.
    if (ah == 0x03) {
        mem_swz_write_h(wram_chip, a & 0x7FFF, value);
        thumb_block_invalidate_page(a);
        return;
    }
    if (ah == 0x02) {
        mem_swz_write_h(wram_board, a & ewram_mask, value);
        thumb_block_invalidate_page(a);
        return;
    }
    // Other page-table regions (OAM keeps the byte-store path).
    mem_page_t *p = &mem_write_pages[ah];
    if (p->base) {
        uint8_t *dst = p->base + (a & p->mask);
        dst[0] = (uint8_t)(value >> 0);
        dst[1] = (uint8_t)(value >> 8);
        return;
    }
    // VRAM (0x06): halfword writes don't have the byte-duplicate quirk;
    // only the mirroring rule applies. Two-byte aligned writes share the
    // same mirror-fold result.
    if ((a >> 24) == 0x06) {
        uint32_t off = a & 0x1FFFF;
        if (off & 0x10000) off &= 0x17FFF;
        vram[off + 0] = (uint8_t)(value >> 0);
        vram[off + 1] = (uint8_t)(value >> 8);
        return;
    }
    // PRAM (0x05): inline the BGR555->RGB565 conversion. The slow path
    // calls pram_write twice (once per byte), each call re-reads
    // palette_ram and re-computes the pixel. Inlining lets us compute the
    // RGB565 entry directly from `value`. Critical for fade scenes where
    // games write all 512 palette entries per frame.
    if ((a >> 24) == 0x05) {
        uint32_t pa = a & 0x3FE;
        palette_ram[pa + 0] = (uint8_t)(value >> 0);
        palette_ram[pa + 1] = (uint8_t)(value >> 8);
        uint32_t r = (value >>  0) & 0x1f;
        uint32_t g = (value >>  5) & 0x1f;
        uint32_t b = (value >> 10) & 0x1f;
        palette[pa >> 1] = (r << 11) | (((g << 1) | (g >> 4)) << 5) | b;
        return;
    }

    arm_write_(a | 0, 0, (uint8_t)(value >> 0));
    arm_write_(a | 1, 1, (uint8_t)(value >> 8));
}

void arm_write(uint32_t address, uint32_t value) {
    uint32_t a = address & ~3;
    uint32_t ah = (a >> 24) & 0xFF;

    // Fast path: EWRAM/IWRAM word writes go to the host-swizzled buffers
    // as a single 32-bit store. Phase 4c: invalidate any cached Thumb
    // block covering this page.
    if (ah == 0x03) {
        mem_swz_write_w(wram_chip, a & 0x7FFF, value);
        thumb_block_invalidate_page(a);
        return;
    }
    if (ah == 0x02) {
        mem_swz_write_w(wram_board, a & ewram_mask, value);
        thumb_block_invalidate_page(a);
        return;
    }
    // Other page-table regions (OAM keeps the byte-store path).
    mem_page_t *p = &mem_write_pages[ah];
    if (p->base) {
        uint8_t *dst = p->base + (a & p->mask);
        dst[0] = (uint8_t)(value >>  0);
        dst[1] = (uint8_t)(value >>  8);
        dst[2] = (uint8_t)(value >> 16);
        dst[3] = (uint8_t)(value >> 24);
        return;
    }
    // VRAM (0x06): word writes share the mirroring rule.
    if ((a >> 24) == 0x06) {
        uint32_t off = a & 0x1FFFF;
        if (off & 0x10000) off &= 0x17FFF;
        vram[off + 0] = (uint8_t)(value >>  0);
        vram[off + 1] = (uint8_t)(value >>  8);
        vram[off + 2] = (uint8_t)(value >> 16);
        vram[off + 3] = (uint8_t)(value >> 24);
        return;
    }
    // PRAM (0x05): one word write covers two palette entries. Inline both
    // BGR555->RGB565 conversions — same motivation as the halfword fast path.
    if ((a >> 24) == 0x05) {
        uint32_t pa = a & 0x3FC;
        uint16_t v0 = (uint16_t)(value >>  0);
        uint16_t v1 = (uint16_t)(value >> 16);
        palette_ram[pa + 0] = (uint8_t)(v0 >> 0);
        palette_ram[pa + 1] = (uint8_t)(v0 >> 8);
        palette_ram[pa + 2] = (uint8_t)(v1 >> 0);
        palette_ram[pa + 3] = (uint8_t)(v1 >> 8);
        uint32_t r0 = (v0 >>  0) & 0x1f;
        uint32_t g0 = (v0 >>  5) & 0x1f;
        uint32_t b0 = (v0 >> 10) & 0x1f;
        uint32_t r1 = (v1 >>  0) & 0x1f;
        uint32_t g1 = (v1 >>  5) & 0x1f;
        uint32_t b1 = (v1 >> 10) & 0x1f;
        palette[(pa >> 1) + 0] = (r0 << 11) | (((g0 << 1) | (g0 >> 4)) << 5) | b0;
        palette[(pa >> 1) + 1] = (r1 << 11) | (((g1 << 1) | (g1 >> 4)) << 5) | b1;
        return;
    }

    arm_write_(a | 0, 0, (uint8_t)(value >>  0));
    arm_write_(a | 1, 1, (uint8_t)(value >>  8));
    arm_write_(a | 2, 2, (uint8_t)(value >> 16));
    arm_write_(a | 3, 3, (uint8_t)(value >> 24));
}

void arm_writeb_n(uint32_t address, uint8_t value) {
    arm_access_bus(address, ARM_BYTE_SZ, NON_SEQ);

    arm_writeb(address, value);
}

void arm_writeh_n(uint32_t address, uint16_t value) {
    arm_access_bus(address, ARM_HWORD_SZ, NON_SEQ);

    arm_writeh(address, value);
}

void arm_write_n(uint32_t address, uint32_t value) {
    arm_access_bus(address, ARM_WORD_SZ, NON_SEQ);

    arm_write(address, value);
}

void arm_writeb_s(uint32_t address, uint8_t value) {
    arm_access_bus(address, ARM_BYTE_SZ, SEQUENTIAL);

    arm_writeb(address, value);
}

void arm_writeh_s(uint32_t address, uint16_t value) {
    arm_access_bus(address, ARM_HWORD_SZ, SEQUENTIAL);

    arm_writeh(address, value);
}

void arm_write_s(uint32_t address, uint32_t value) {
    arm_access_bus(address, ARM_WORD_SZ, SEQUENTIAL);

    arm_write(address, value);
}