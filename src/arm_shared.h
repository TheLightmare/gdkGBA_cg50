#ifndef ARM_SHARED_H
#define ARM_SHARED_H

#include <stdint.h>
#include <stdbool.h>
#include "rom_buffer.h"
#include "arm_mem.h" // Include this for access_type_e

// Memory size constants
#define ARM_BYTE_SZ  1
#define ARM_HWORD_SZ 2
#define ARM_WORD_SZ  4

// Memory areas
extern uint8_t bios[16384];
extern uint8_t *wram_board;             // wram in arm.c, dynamic 64K or 256K
extern uint32_t ewram_mask;
extern uint32_t ewram_size;
extern uint8_t wram_chip[0x00008000];   // iwram in arm.c
extern uint8_t palette_ram[0x00000400]; // pram in arm.c
extern uint8_t vram[0x00018000];
extern uint8_t oam[0x00000400];

// External memory
extern uint8_t *eeprom;
extern uint8_t *sram;
extern uint8_t *flash;

// ROM buffer
extern RomBuffer rom_buffer;

// Other memory-related variables
extern uint32_t bios_op;
extern uint32_t cart_rom_size;
extern uint32_t cart_rom_mask;
extern uint16_t palette[0x200];

// Memory access functions
uint8_t arm_readb_n(uint32_t address);
uint32_t arm_readh_n(uint32_t address);
uint32_t arm_read_n(uint32_t address);

uint8_t arm_readb_s(uint32_t address);
uint32_t arm_readh_s(uint32_t address);
uint32_t arm_read_s(uint32_t address);

void arm_writeb_n(uint32_t address, uint8_t value);
void arm_writeh_n(uint32_t address, uint16_t value);
void arm_write_n(uint32_t address, uint32_t value);

void arm_writeb_s(uint32_t address, uint8_t value);
void arm_writeh_s(uint32_t address, uint16_t value);
void arm_write_s(uint32_t address, uint32_t value);

#endif // ARM_SHARED_H
