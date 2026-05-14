#include "sh4_emit.h"

#include <stdint.h>
#include <stddef.h>

// SH4 instruction encoders. Bit layouts come straight from the SH4 ISA
// reference. Every helper appends exactly one 16-bit instruction via
// jit_emit16(). Register parameters are masked to 4 bits.

#define R(r)  ((uint16_t)((r) & 0xF))

// ----- Plain register moves and stack push/pop -----------------------------

void emit_mov_rr(uint8_t rm, uint8_t rn) {
    // 0110 nnnn mmmm 0011
    jit_emit16((uint16_t)(0x6003 | (R(rn) << 8) | (R(rm) << 4)));
}

void emit_pushl_rr(uint8_t rm, uint8_t rn) {
    // mov.l Rm, @-Rn   :   0010 nnnn mmmm 0110
    jit_emit16((uint16_t)(0x2006 | (R(rn) << 8) | (R(rm) << 4)));
}

void emit_popl_rr(uint8_t rm, uint8_t rn) {
    // mov.l @Rm+, Rn   :   0110 nnnn mmmm 0110
    jit_emit16((uint16_t)(0x6006 | (R(rn) << 8) | (R(rm) << 4)));
}

void emit_sts_l_pr(uint8_t rn) {
    // sts.l PR, @-Rn   :   0100 nnnn 0010 0010
    jit_emit16((uint16_t)(0x4022 | (R(rn) << 8)));
}

void emit_lds_l_pr(uint8_t rn) {
    // lds.l @Rn+, PR   :   0100 nnnn 0010 0110
    jit_emit16((uint16_t)(0x4026 | (R(rn) << 8)));
}

// ----- Loads and stores ----------------------------------------------------

void emit_movl_disp_rm_rn(uint8_t disp_longs, uint8_t rm, uint8_t rn) {
    // mov.l @(disp, Rm), Rn   :   0101 nnnn mmmm dddd  (disp_longs 0..15)
    jit_emit16((uint16_t)(0x5000 | (R(rn) << 8) | (R(rm) << 4) | (disp_longs & 0xF)));
}

void emit_movl_rm_disp_rn(uint8_t rm, uint8_t disp_longs, uint8_t rn) {
    // mov.l Rm, @(disp, Rn)   :   0001 nnnn mmmm dddd
    jit_emit16((uint16_t)(0x1000 | (R(rn) << 8) | (R(rm) << 4) | (disp_longs & 0xF)));
}

void emit_movb_at_rm_rn(uint8_t rm, uint8_t rn) {
    // mov.b @Rm, Rn   :   0110 nnnn mmmm 0000
    jit_emit16((uint16_t)(0x6000 | (R(rn) << 8) | (R(rm) << 4)));
}

void emit_movb_rm_at_rn(uint8_t rm, uint8_t rn) {
    // mov.b Rm, @Rn   :   0010 nnnn mmmm 0000
    jit_emit16((uint16_t)(0x2000 | (R(rn) << 8) | (R(rm) << 4)));
}

void emit_extub_rm_rn(uint8_t rm, uint8_t rn) {
    // extu.b Rm, Rn   :   0110 nnnn mmmm 1100
    jit_emit16((uint16_t)(0x600C | (R(rn) << 8) | (R(rm) << 4)));
}

// ----- Immediates and arithmetic -------------------------------------------

void emit_mov_imm_rn(int8_t imm, uint8_t rn) {
    // mov #imm, Rn   :   1110 nnnn iiiiiiii   (imm sign-extended at exec)
    jit_emit16((uint16_t)(0xE000 | (R(rn) << 8) | (uint8_t)imm));
}

void emit_add_imm_rn(int8_t imm, uint8_t rn) {
    // add #imm, Rn   :   0111 nnnn iiiiiiii
    jit_emit16((uint16_t)(0x7000 | (R(rn) << 8) | (uint8_t)imm));
}

void emit_add_rr(uint8_t rm, uint8_t rn) {
    // add Rm, Rn   :   0011 nnnn mmmm 1100   (no flags; T unchanged)
    jit_emit16((uint16_t)(0x300C | (R(rn) << 8) | (R(rm) << 4)));
}

void emit_tst_rr(uint8_t rm, uint8_t rn) {
    // tst Rm, Rn   :   0010 nnnn mmmm 1000
    jit_emit16((uint16_t)(0x2008 | (R(rn) << 8) | (R(rm) << 4)));
}

// ----- Control flow --------------------------------------------------------

void emit_jsr_at_rn(uint8_t rn) {
    // jsr @Rn   :   0100 nnnn 0000 1011   (delay slot follows)
    jit_emit16((uint16_t)(0x400B | (R(rn) << 8)));
}

void emit_rts(void) {
    // rts   :   0000 0000 0000 1011   (delay slot follows)
    jit_emit16(0x000B);
}

void emit_nop(void) {
    jit_emit16(0x0009);
}

void emit_bf(int16_t disp_insts) {
    // bf disp   :   1000 1011 dddddddd   (no delay slot; disp signed 8b)
    jit_emit16((uint16_t)(0x8B00 | (uint8_t)(int8_t)disp_insts));
}

void emit_bt(int16_t disp_insts) {
    // bt disp   :   1000 1001 dddddddd
    jit_emit16((uint16_t)(0x8900 | (uint8_t)(int8_t)disp_insts));
}

void emit_bra(int16_t disp_insts) {
    // bra disp   :   1010 dddddddddddd   (delay slot follows; disp signed 12b)
    jit_emit16((uint16_t)(0xA000 | ((uint16_t)disp_insts & 0x0FFF)));
}

// ----- Literal-pool fix-ups ------------------------------------------------

typedef struct {
    uint16_t *movl_pos;
    uint32_t  value;
} sh4_lit_fixup_t;

static sh4_lit_fixup_t lit_fixups[SH4_LIT_POOL_MAX];
static uint16_t lit_count = 0;

void emit_movl_pc_lit_reset(void) {
    lit_count = 0;
}

bool emit_movl_pc_lit(uint32_t value, uint8_t rn) {
    if (lit_count >= SH4_LIT_POOL_MAX) return false;
    // mov.l @(disp, PC), Rn   :   1101 nnnn dddddddd
    // Emit with disp=0; the real value gets patched in by
    // emit_literal_pool_finalize once the literal's address is known.
    jit_emit16((uint16_t)(0xD000 | (R(rn) << 8)));
    lit_fixups[lit_count].movl_pos = jit_cursor() - 1;
    lit_fixups[lit_count].value    = value;
    lit_count++;
    return true;
}

bool emit_literal_pool_finalize(void) {
    // Align cursor to 4 bytes. The pool sits past the rts; nop, so any
    // padding here is unreachable and a nop is the safe filler.
    if (((uintptr_t)jit_cursor() & 2u) != 0u) {
        emit_nop();
    }

    for (uint16_t i = 0; i < lit_count; i++) {
        uint16_t *lit_pos = jit_cursor();
        // Store the 32-bit value as two big-endian halfwords. SH4 mov.l
        // reads it back as a 32-bit aligned long.
        uint32_t v = lit_fixups[i].value;
        jit_emit16((uint16_t)((v >> 16) & 0xFFFF));
        jit_emit16((uint16_t)( v        & 0xFFFF));

        // Compute the disp byte that the placeholder movl needs. SH4
        // mov.l @(disp, PC), Rn reads from ((movl_pc + 4) & ~3) + disp*4.
        uintptr_t movl_addr       = (uintptr_t)lit_fixups[i].movl_pos;
        uintptr_t movl_pc_aligned = (movl_addr + 4u) & ~(uintptr_t)3u;
        uintptr_t target          = (uintptr_t)lit_pos;
        if (target < movl_pc_aligned) return false;   // shouldn't happen
        uintptr_t offset_bytes = target - movl_pc_aligned;
        if (offset_bytes & 3u)   return false;        // misaligned literal
        uintptr_t disp = offset_bytes >> 2;
        if (disp > 255u)         return false;        // 8-bit disp overflow

        uint16_t old = *lit_fixups[i].movl_pos;
        *lit_fixups[i].movl_pos = (uint16_t)((old & 0xFF00) | (disp & 0xFFu));
    }
    lit_count = 0;
    return true;
}
