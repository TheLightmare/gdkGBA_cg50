#ifndef SH4_EMIT_H
#define SH4_EMIT_H

#include <stdint.h>
#include <stdbool.h>

#include "sh4_jit.h"

// Low-level SH4 instruction encoders. Every emit_* function appends one
// 16-bit instruction to the JIT arena via jit_emit16(). The caller is
// responsible for having called jit_emit_begin() and tracking the
// budget. Encodings come straight from the SH4 ISA manual; helpers
// are named after the assembler mnemonic.
//
// Register parameters are 4-bit values (0..15). The helpers do not
// validate; passing a value > 15 will corrupt adjacent bits of the
// encoding.

// ----- Plain register moves and stack push/pop -----------------------------

void emit_mov_rr     (uint8_t rm, uint8_t rn);     // mov Rm, Rn
void emit_pushl_rr   (uint8_t rm, uint8_t rn);     // mov.l Rm, @-Rn   (predecrement)
void emit_popl_rr    (uint8_t rm, uint8_t rn);     // mov.l @Rm+, Rn   (postincrement)
void emit_sts_l_pr   (uint8_t rn);                  // sts.l PR, @-Rn
void emit_lds_l_pr   (uint8_t rn);                  // lds.l @Rn+, PR

// ----- Loads and stores ----------------------------------------------------

// mov.l @(disp, Rm), Rn   -- disp is the LONG index 0..15 (byte offset = disp*4)
void emit_movl_disp_rm_rn(uint8_t disp_longs, uint8_t rm, uint8_t rn);

// mov.l Rm, @(disp, Rn)
void emit_movl_rm_disp_rn(uint8_t rm, uint8_t disp_longs, uint8_t rn);

// mov.b @Rm, Rn ; mov.b Rm, @Rn
void emit_movb_at_rm_rn(uint8_t rm, uint8_t rn);
void emit_movb_rm_at_rn(uint8_t rm, uint8_t rn);

// extu.b Rm, Rn   -- zero-extend the low byte of Rm into Rn
void emit_extub_rm_rn(uint8_t rm, uint8_t rn);

// ----- Immediates and arithmetic -------------------------------------------

void emit_mov_imm_rn(int8_t imm, uint8_t rn);       // mov #imm, Rn   (signed 8)
void emit_add_imm_rn(int8_t imm, uint8_t rn);       // add #imm, Rn   (signed 8)
void emit_tst_rr    (uint8_t rm, uint8_t rn);        // tst Rm, Rn

// ----- Control flow --------------------------------------------------------

void emit_jsr_at_rn(uint8_t rn);   // jsr @Rn -- delay slot follows
void emit_rts      (void);          // rts     -- delay slot follows
void emit_nop      (void);

// Conditional/unconditional short branches. `disp_insts` is the count of
// 16-bit instruction slots from the slot AFTER the branch to the target,
// i.e. target = (branch_pc + 4) + (disp_insts * 2). Signed 8-bit
// (-128..+127) for bf/bt, signed 12-bit (-2048..+2047) for bra.
void emit_bf  (int16_t disp_insts);   // branch if T == 0
void emit_bt  (int16_t disp_insts);   // branch if T == 1
void emit_bra (int16_t disp_insts);   // unconditional, delay slot follows

// ----- Literal-pool fix-ups ------------------------------------------------
//
// jit_emit_movl_pc_lit() reserves a `mov.l @(disp,PC), Rn` slot with a
// placeholder disp byte, and remembers (cursor position, target value)
// in a per-block fixup table. After all code for the block has been
// emitted, call sh4_emit_literal_pool() to drop the literal values at
// the next 4-aligned cursor position and patch every recorded movl.
//
// Returns false if the fixup table is full -- callers must check and
// bail (typically by skipping codegen for an overly-large block).

// Sized to cover a max-length Thumb block: 2 literals per uop (uop ptr
// + handler) plus a handful for global addresses (pipe_reload,
// arm_load_pipe). 2 * THUMB_BLOCK_MAX_LEN + 8 with headroom.
#define SH4_LIT_POOL_MAX 160

void emit_movl_pc_lit_reset(void);
bool emit_movl_pc_lit(uint32_t value, uint8_t rn);

// Emit the recorded literals at the current cursor and patch the
// placeholder mov.l instructions. Returns false if any fixup's disp
// would overflow the 8-bit field (block too long); on failure the
// arena cursor is left where it is and the block must be discarded.
bool emit_literal_pool_finalize(void);

#endif
