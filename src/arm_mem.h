#ifndef ARM_MEM_H
#define ARM_MEM_H

#include <stdint.h>
#include "rom_buffer.h"

// ROM access
extern RomBuffer rom_buffer;

// Original ROM buffer (replace with ROM buffer system)
// extern uint8_t *rom;
extern uint32_t cart_rom_size;
extern uint32_t cart_rom_mask;

// BIOS
extern uint8_t bios[16384];
extern uint32_t bios_mask;

// WRAM
// EWRAM is dynamically sized: 256 KB (full GBA spec) when allocated from
// the optional "extram" arena, 64 KB (clamped, with mirroring) otherwise.
// See ewram_init() in arm_mem.c.
extern uint8_t *wram_board;
extern uint32_t ewram_mask;
extern uint32_t ewram_size;
void ewram_init(void);
extern uint8_t wram_chip[0x00008000];

extern uint8_t palette_ram[0x00000400];
extern uint16_t palette[0x200];
extern uint8_t vram[0x00018000];
extern uint8_t oam[0x00000400];

// EEPROM
extern uint16_t eeprom_idx;

// key_input lives in io.c; the io_reg union (defined in io.h) is the canonical
// type — see io.h. The forward declaration is kept here for callers that pull
// in arm_mem.h alongside io.h.
#include "io.h"

uint8_t mem_read_8(uint32_t address);
uint16_t mem_read_16(uint32_t address);
uint32_t mem_read_32(uint32_t address);

void mem_write_8(uint32_t address, uint8_t value);
void mem_write_16(uint32_t address, uint16_t value);
void mem_write_32(uint32_t address, uint32_t value);

// Page table for the read fast path. One entry per 16MB region (top byte of
// the address). base==NULL means "fall through to the slow path"; otherwise
// the byte at `base[addr & mask]` is the read result. Built once at boot by
// arm_mem_pages_init() (called from ewram_init).
typedef struct {
    uint8_t  *base;
    uint32_t  mask;
} mem_page_t;

extern mem_page_t mem_read_pages[256];

// Slow paths exposed so the inline fast-path branches can tail-call them.
uint8_t  arm_readb_slow(uint32_t address);
uint32_t arm_readh_slow(uint32_t address);
uint32_t arm_read_slow(uint32_t address);

// Define access_type_e and constants
typedef enum {
    SEQUENTIAL,
    NON_SEQ
} access_type_e;

// Regular memory access
uint32_t arm_read(uint32_t address);
void arm_write(uint32_t address, uint32_t value);

// Access with type specification
uint32_t arm_read_n(uint32_t address);  // Non-sequential
uint32_t arm_read_s(uint32_t address);  // Sequential
void arm_write_n(uint32_t address, uint32_t value);
void arm_write_s(uint32_t address, uint32_t value);

// Halfword access (returns uint32_t because the value may be ROR-rotated by
// the misalignment shift performed inside arm_readh).
uint32_t arm_readh(uint32_t address);
void arm_writeh(uint32_t address, uint16_t value);
uint32_t arm_readh_n(uint32_t address);
uint32_t arm_readh_s(uint32_t address);
void arm_writeh_n(uint32_t address, uint16_t value);
void arm_writeh_s(uint32_t address, uint16_t value);

// Byte access
uint8_t arm_readb(uint32_t address);
void arm_writeb(uint32_t address, uint8_t value);
uint8_t arm_readb_n(uint32_t address);
uint8_t arm_readb_s(uint32_t address);
void arm_writeb_n(uint32_t address, uint8_t value);
void arm_writeb_s(uint32_t address, uint8_t value);

#endif