#ifndef ARM_MEM_H
#define ARM_MEM_H

#include <stdint.h>

extern uint8_t *bios;
extern uint8_t *wram;
extern uint8_t *iwram;
extern uint8_t *pram;
extern uint8_t *vram;
extern uint8_t *oam;
extern uint8_t *rom;
extern uint8_t *eeprom;
extern uint8_t *sram;
extern uint8_t *flash;

extern uint32_t palette[0x200];

extern uint32_t bios_op;

extern int64_t cart_rom_size;
extern uint32_t cart_rom_mask;

extern uint16_t eeprom_idx;

typedef enum {
    NON_SEQ,
    SEQUENTIAL
} access_type_e;

uint8_t arm_readb(uint32_t address);
uint32_t arm_readh(uint32_t address);
uint32_t arm_read(uint32_t address);
uint8_t arm_readb_n(uint32_t address);
uint32_t arm_readh_n(uint32_t address);
uint32_t arm_read_n(uint32_t address);
uint8_t arm_readb_s(uint32_t address);
uint32_t arm_readh_s(uint32_t address);
uint32_t arm_read_s(uint32_t address);

void arm_writeb(uint32_t address, uint8_t value);
void arm_writeh(uint32_t address, uint16_t value);
void arm_write(uint32_t address, uint32_t value);
void arm_writeb_n(uint32_t address, uint8_t value);
void arm_writeh_n(uint32_t address, uint16_t value);
void arm_write_n(uint32_t address, uint32_t value);
void arm_writeb_s(uint32_t address, uint8_t value);
void arm_writeh_s(uint32_t address, uint16_t value);
void arm_write_s(uint32_t address, uint32_t value);

#endif