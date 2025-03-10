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
extern uint8_t wram_board[0x00010000];
extern uint8_t wram_chip[0x00008000];

extern uint8_t palette_ram[0x00000400];
extern uint8_t vram[0x00018000];
extern uint8_t oam[0x00000400];

// IO stuff
typedef union {
    struct { uint16_t l, h; };
    uint32_t w;
} reg32_t;

extern reg32_t key_input;

uint8_t mem_read_8(uint32_t address);
uint16_t mem_read_16(uint32_t address);
uint32_t mem_read_32(uint32_t address);

void mem_write_8(uint32_t address, uint8_t value);
void mem_write_16(uint32_t address, uint16_t value);
void mem_write_32(uint32_t address, uint32_t value);

#endif