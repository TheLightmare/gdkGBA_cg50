#include "arm_jit.h"

#include <stdint.h>
#include <stddef.h>

#include "arm.h"            // pipe_reload, arm_load_pipe
#include "arm_block.h"
#include "bench.h"
#include "build_flags.h"
#include "sh4_emit.h"
#include "sh4_jit.h"

// Phase 1 Chunk 4: ARM block JIT framework. Mirror of thumb_jit.c but
// for ARM blocks (4-byte instructions; R15 step of +4 per uop instead
// of +2; sequential-exit fix-up of -4 instead of -2). Every uop is
// emitted as a JSR to its existing C handler -- no opcode specialisation
// yet. The point of this chunk is to validate the codegen on ARM mode
// (the heavier 40 ms/frame budget) before any per-opcode inlining.
//
// Register usage mirrors the Thumb side:
//   r0    scratch (literal targets, handler call target)
//   r1    scratch (pipe_reload byte)
//   r4    first arg to legacy handler (uop ptr); reloaded per uop
//   r8    &arm_r for the lifetime of the block; saved on entry
//   r15   stack pointer
//   PR    saved on entry, restored on exit
//
// Decoder guarantee: arm_block_decode terminates a block at branches,
// SWIs and any other PC-write opcode, so only the LAST uop can set
// pipe_reload. The per-uop check is therefore skipped and we make one
// check at block end.

#define ARM_R15_LONG_DISP 15

// Bench counters (mirror of the Thumb-side trackers). Declared local
// to avoid touching bench.h until we settle on a permanent name.
extern uint32_t bench_arm_jit_compiled;
extern uint32_t bench_arm_jit_specialized_ops;

// Handlers we specialise inline. Declared locally to avoid pulling
// every arm_dec_* declaration into the header.
extern void arm_dec_b_imm24    (const arm_uop_t *uop);
extern void arm_dec_bl_imm24   (const arm_uop_t *uop);
extern void arm_dec_mov_imm    (const arm_uop_t *uop);
extern void arm_dec_call_legacy(const arm_uop_t *uop);

// Phase 2 chunk 3 flag-update helpers. Called from JIT'd blocks via JSR
// after r4/r5 are loaded with the (Rn, Rm) operand values. CMP/CMN/TST/TEQ
// return nothing (flags only); ADDS/SUBS return the result in r0 so the
// caller can store it into Rd.
extern void     arm_jit_cmp (uint32_t lhs, uint32_t rhs);
extern void     arm_jit_cmn (uint32_t lhs, uint32_t rhs);
extern void     arm_jit_tst (uint32_t lhs, uint32_t rhs);
extern void     arm_jit_teq (uint32_t lhs, uint32_t rhs);
extern uint32_t arm_jit_adds(uint32_t lhs, uint32_t rhs);
extern uint32_t arm_jit_subs(uint32_t lhs, uint32_t rhs);

// Phase 2 chunk 4 shifter helpers (value-only; arm_flag_c unchanged).
extern uint32_t arm_jit_lsl_v(uint32_t val, uint32_t sh);
extern uint32_t arm_jit_lsr_v(uint32_t val, uint32_t sh);
extern uint32_t arm_jit_asr_v(uint32_t val, uint32_t sh);
extern uint32_t arm_jit_ror_v(uint32_t val, uint32_t sh);

// Phase 2 chunk 5 cout-shifter helpers (write arm_flag_c to shifter-out).
extern uint32_t arm_jit_lsl_vc(uint32_t val, uint32_t sh);
extern uint32_t arm_jit_lsr_vc(uint32_t val, uint32_t sh);
extern uint32_t arm_jit_asr_vc(uint32_t val, uint32_t sh);
extern uint32_t arm_jit_ror_vc(uint32_t val, uint32_t sh);

// Phase 2 chunk 5 S=1 logic-op helpers (set N/Z, leave C from shifter).
extern uint32_t arm_jit_ands(uint32_t lhs, uint32_t rhs);
extern uint32_t arm_jit_eors(uint32_t lhs, uint32_t rhs);
extern uint32_t arm_jit_orrs(uint32_t lhs, uint32_t rhs);
extern uint32_t arm_jit_bics(uint32_t lhs, uint32_t rhs);
extern uint32_t arm_jit_movs(uint32_t lhs_unused, uint32_t rhs);
extern uint32_t arm_jit_mvns(uint32_t lhs_unused, uint32_t rhs);

// Phase 2 chunk 6: data-proc imm uses arm_flag_c directly for cout staging
// (S=1 logic ops). arm_flag_c is a global defined in arm.c.
extern uint32_t arm_flag_c;

static size_t estimate_block_bytes(uint16_t length) {
    // Worst case per uop: cond-checked BL imm24 = 6 (cond prefix) +
    // 12 (body) = 18 SH4 instructions = 36 B + 3 literals = ~48 B.
    // Use ~50 B/uop as a conservative bound. Post-block ~28 B; literal
    // headroom for globals plus padding.
    size_t code = 6u + 50u * (size_t)length + 36u;
    size_t lits = (3u * (size_t)length + 2u) * 4u + 2u;
    return code + lits + 32u;
}

static bool emit_jsr_fallback(const arm_uop_t *uop) {
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)uop,          4)) return false;
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)uop->handler, 0)) return false;
    emit_jsr_at_rn(0);
    emit_nop();
    return true;
}

static void emit_arm_r15_add(int8_t delta) {
    emit_movl_disp_rm_rn(ARM_R15_LONG_DISP, 8, 0);   // r0 = arm_r.r[15]
    emit_add_imm_rn(delta, 0);                       // r0 += delta
    emit_movl_rm_disp_rn(0, ARM_R15_LONG_DISP, 8);   // arm_r.r[15] = r0
}

// ----- ARM specialisations -------------------------------------------------
//
// ARM ops carry a 4-bit condition. cond=14 (AL) runs unconditionally;
// cond=15 (UNCOND) only appears for the legacy fallback path; cond
// 0..13 require a runtime arm_cond_check JSR. The cond is known at
// codegen time, so we branch on the codegen side: cond=14 emits body
// only, cond 0..13 wraps the body in a cond-check + bt-skip.

// Emit r4 = cond; JSR arm_cond_check; tst r0, r0. After this, T=1 if
// the cond evaluated FALSE (r0 == 0), and T=0 if TRUE. A bt with the
// body's size will skip the body when T=1 (cond false).
// Cost: 6 SH4 instructions + 1 literal.
static bool emit_arm_cond_check_prefix(uint8_t cond) {
    emit_mov_imm_rn((int8_t)cond, 4);                               // r4 = cond
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)&arm_cond_check, 0)) // r0 = &arm_cond_check
        return false;
    emit_jsr_at_rn(0);
    emit_nop();
    emit_tst_rr(0, 0);                                              // T = (r0 == 0)
    return true;
}

// Body: r15 += imm; r15 &= ~3; arm_load_pipe().
// ARM B imm24 -> arg_c always sign-extended (imm24 << 2), exceeds int8
// signed range in nearly every case so we always use the literal-pool
// path for the imm. 9 SH4 instructions + 2 literals.
static bool emit_arm_b_body(int32_t imm) {
    emit_movl_disp_rm_rn(ARM_R15_LONG_DISP, 8, 0);     // r0 = arm_r.r[15]
    if (!emit_movl_pc_lit((uint32_t)imm, 1)) return false;
    emit_add_rr(1, 0);                                  // r0 += imm
    emit_shlr2(0);                                      // r0 &= ~3 via shifts
    emit_shll2(0);
    emit_movl_rm_disp_rn(0, ARM_R15_LONG_DISP, 8);     // arm_r.r[15] = r0
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)&arm_load_pipe, 0)) return false;
    emit_jsr_at_rn(0);
    emit_nop();
    return true;
}

// arm_dec_b_imm24 spec: optional cond check + the body above.
static bool emit_arm_b_imm24(uint8_t cond, int32_t imm) {
    if (cond == 14) {
        return emit_arm_b_body(imm);
    }
    // Conditional: prefix + bt-skip 9-inst body. disp_insts = 8.
    if (!emit_arm_cond_check_prefix(cond)) return false;
    emit_bt(8);
    return emit_arm_b_body(imm);
}

// ARM BL imm24: same as B + LR write (R14 = R15 - 4) BEFORE incrementing
// R15. 12 SH4 instructions + 2 literals.
static bool emit_arm_bl_body(int32_t imm) {
    emit_movl_disp_rm_rn(ARM_R15_LONG_DISP, 8, 0);     // r0 = arm_r.r[15]
    emit_mov_rr(0, 1);                                  // r1 = r0  (preserve)
    emit_add_imm_rn(-4, 1);                             // r1 -= 4
    emit_movl_rm_disp_rn(1, 14, 8);                     // arm_r.r[14] = r1 (LR)
    if (!emit_movl_pc_lit((uint32_t)imm, 1)) return false;
    emit_add_rr(1, 0);                                  // r0 += imm
    emit_shlr2(0);
    emit_shll2(0);
    emit_movl_rm_disp_rn(0, ARM_R15_LONG_DISP, 8);     // arm_r.r[15] = r0
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)&arm_load_pipe, 0)) return false;
    emit_jsr_at_rn(0);
    emit_nop();
    return true;
}

static bool emit_arm_bl_imm24(uint8_t cond, int32_t imm) {
    if (cond == 14) {
        return emit_arm_bl_body(imm);
    }
    if (!emit_arm_cond_check_prefix(cond)) return false;
    emit_bt(11);                            // skip 12-inst body
    return emit_arm_bl_body(imm);
}

// ARM MOV Rd, #imm (no S-bit): arm_r.r[Rd] = imm. 2 SH4 instructions
// + 1 literal. Decoder gates Rd==15 to legacy, so no PC writes here.
static bool emit_arm_mov_imm_body(uint8_t rd, uint32_t imm) {
    if (!emit_movl_pc_lit(imm, 0)) return false;
    emit_movl_rm_disp_rn(0, (uint8_t)(rd & 0xF), 8);
    return true;
}

static bool emit_arm_mov_imm(uint8_t cond, uint8_t rd, uint32_t imm) {
    if (cond == 14) {
        return emit_arm_mov_imm_body(rd, imm);
    }
    if (!emit_arm_cond_check_prefix(cond)) return false;
    emit_bt(1);                             // skip 2-inst body
    return emit_arm_mov_imm_body(rd, imm);
}

// Phase 2 Chunk 1: inline ARM data-proc REG form, no-shift / S=0 subset.
//
// The interpreter has no specialised handler for these -- they all
// route through arm_dec_call_legacy. The JIT recognises the pattern
// straight from uop->raw_op and emits inline native code, which
// bypasses the whole arm_proc_handlers[] dispatch + helper-call cost
// the legacy path pays per op.
//
// Pattern (cond field already in uop->cond):
//   bits 27..25 = 0b000     -- data-proc / misc group
//   bits 11..4  = 0b00000000-- LSL #0 (no shift), imm-shift form
//   bit 20      = 0         -- S=0 (flag-update variants deferred to chunk 3)
//   opc (24..21) in supported set (excludes ADC/SBC/RSC -- need C flag,
//                                  excludes TST/TEQ/CMP/CMN -- always S=1)
//   Rd (15..12) != 15       -- data-proc Rd=15 ends the block; let the
//                              legacy path drive pipe_reload
//
// Body register usage:
//   r0 = first operand / result accumulator
//   r1 = second operand / scratch
// All loads/stores go through r8 = &arm_r as elsewhere.

static int emit_arm_dp_reg_size(uint8_t opc) {
    switch (opc) {
        case 0xD: return 2;   // MOV  Rd, Rm
        case 0xF: return 3;   // MVN  Rd, Rm
        case 0xE: return 5;   // BIC  Rd, Rn, Rm
        default:  return 4;   // AND/EOR/SUB/RSB/ADD/ORR
    }
}

static void emit_arm_dp_reg_body(uint8_t opc, uint8_t rn, uint8_t rd, uint8_t rm) {
    if (opc == 0xD) {                      // MOV Rd, Rm
        emit_movl_disp_rm_rn(rm, 8, 0);
        emit_movl_rm_disp_rn(0, rd, 8);
        return;
    }
    if (opc == 0xF) {                      // MVN Rd, Rm
        emit_movl_disp_rm_rn(rm, 8, 0);
        emit_not_rr(0, 0);
        emit_movl_rm_disp_rn(0, rd, 8);
        return;
    }
    // 3-operand: load Rn into r0, Rm into r1.
    emit_movl_disp_rm_rn(rn, 8, 0);
    emit_movl_disp_rm_rn(rm, 8, 1);

    switch (opc) {
        case 0x0:  // AND  Rd = Rn & Rm
            emit_and_rr(1, 0);
            break;
        case 0x1:  // EOR  Rd = Rn ^ Rm
            emit_xor_rr(1, 0);
            break;
        case 0x2:  // SUB  Rd = Rn - Rm
            emit_sub_rr(1, 0);
            break;
        case 0x3:  // RSB  Rd = Rm - Rn   (sub Rn from Rm, leave result in r1)
            emit_sub_rr(0, 1);
            emit_movl_rm_disp_rn(1, rd, 8);
            return;
        case 0x4:  // ADD  Rd = Rn + Rm
            emit_add_rr(1, 0);
            break;
        case 0xC:  // ORR  Rd = Rn | Rm
            emit_or_rr(1, 0);
            break;
        case 0xE:  // BIC  Rd = Rn & ~Rm
            emit_not_rr(1, 1);
            emit_and_rr(1, 0);
            break;
    }
    emit_movl_rm_disp_rn(0, rd, 8);
}

static int  emit_arm_dp_reg_s1_size(uint8_t opc);
static bool emit_arm_dp_reg_s1_body(uint8_t opc, uint8_t rn, uint8_t rd, uint8_t rm);
static int  emit_arm_dp_reg_shifted_size(uint8_t opc, int kind);
static bool emit_arm_dp_reg_shifted_body(uint8_t opc, uint8_t rn, uint8_t rd,
                                         uint8_t rm, uint8_t shift_type,
                                         uint8_t shift_imm, int kind);

static bool emit_arm_dp_reg(uint8_t cond, uint32_t op, int kind) {
    uint8_t opc        = (op >> 21) & 0xF;
    uint8_t rn         = (op >> 16) & 0xF;
    uint8_t rd         = (op >> 12) & 0xF;
    uint8_t rm         =  op        & 0xF;
    uint8_t shift_imm  = (op >> 7)  & 0x1F;
    uint8_t shift_type = (op >> 5)  & 0x3;

    int body_size;
    switch (kind) {
        case 2:  body_size = emit_arm_dp_reg_s1_size(opc);              break;
        case 3:
        case 4:  body_size = emit_arm_dp_reg_shifted_size(opc, kind);   break;
        default: body_size = emit_arm_dp_reg_size(opc);                 break;
    }

    if (cond != 14) {
        if (!emit_arm_cond_check_prefix(cond)) return false;
        emit_bt((int16_t)(body_size - 1));
    }

    switch (kind) {
        case 2:
            return emit_arm_dp_reg_s1_body(opc, rn, rd, rm);
        case 3:
        case 4:
            return emit_arm_dp_reg_shifted_body(opc, rn, rd, rm,
                                                shift_type, shift_imm, kind);
        default:
            emit_arm_dp_reg_body(opc, rn, rd, rm);
            return true;
    }
}

// Phase 2 chunk 3: helper-call emit for S=1 flag-update variants.
// Shared shape: load Rn into r4, load Rm into r5, jsr to helper.
// ADDS/SUBS take a final mov.l r0, @(Rd,r8) to store the result.
//
// Sizes:
//   Compare-only (CMP/CMN/TST/TEQ):       5 SH4 insts + 1 literal
//   Arith-with-result (ADDS/SUBS):        6 SH4 insts + 1 literal
//
// The literal-pool budget is checked once via emit_movl_pc_lit, which
// returns false if the pool is full. Body size below feeds the
// cond-prefix bt-skip displacement.

static void *arm_dp_reg_s1_helper(uint8_t opc) {
    switch (opc) {
        case 0x0: return (void *)&arm_jit_ands;   // ANDS Rd, Rn, Rm
        case 0x1: return (void *)&arm_jit_eors;   // EORS Rd, Rn, Rm
        case 0x2: return (void *)&arm_jit_subs;   // SUBS Rd, Rn, Rm
        case 0x4: return (void *)&arm_jit_adds;   // ADDS Rd, Rn, Rm
        case 0x8: return (void *)&arm_jit_tst;    // TST Rn, Rm   (no Rd)
        case 0x9: return (void *)&arm_jit_teq;    // TEQ Rn, Rm   (no Rd)
        case 0xA: return (void *)&arm_jit_cmp;    // CMP Rn, Rm   (no Rd)
        case 0xB: return (void *)&arm_jit_cmn;    // CMN Rn, Rm   (no Rd)
        case 0xC: return (void *)&arm_jit_orrs;   // ORRS Rd, Rn, Rm
        case 0xD: return (void *)&arm_jit_movs;   // MOVS Rd, Rm   (Rn ignored)
        case 0xE: return (void *)&arm_jit_bics;   // BICS Rd, Rn, Rm
        case 0xF: return (void *)&arm_jit_mvns;   // MVNS Rd, Rm   (Rn ignored)
        default:  return NULL;
    }
}

static bool opc_writes_dest_s1(uint8_t opc) {
    // TST/TEQ/CMP/CMN do not write Rd. All other S=1 ops do.
    return !(opc == 0x8 || opc == 0x9 || opc == 0xA || opc == 0xB);
}

// Phase 2 chunk 5: S=1 logic ops use the cout-shifter so arm_flag_c
// reflects the shifter output. S=1 arith ops (ADDS/SUBS/CMP/CMN) compute
// C from arithmetic; the shifter cout is irrelevant for them.
static bool opc_s1_uses_shifter_cout(uint8_t opc) {
    switch (opc) {
        case 0x0:  // ANDS
        case 0x1:  // EORS
        case 0x8:  // TST
        case 0x9:  // TEQ
        case 0xC:  // ORRS
        case 0xD:  // MOVS
        case 0xE:  // BICS
        case 0xF:  // MVNS
            return true;
        default:
            return false;
    }
}

static int emit_arm_dp_reg_s1_size(uint8_t opc) {
    return opc_writes_dest_s1(opc) ? 6 : 5;
}

static bool emit_arm_dp_reg_s1_body(uint8_t opc, uint8_t rn, uint8_t rd, uint8_t rm) {
    void *helper = arm_dp_reg_s1_helper(opc);
    if (!helper) return false;

    emit_movl_disp_rm_rn(rn, 8, 4);                       // r4 = Rn
    emit_movl_disp_rm_rn(rm, 8, 5);                       // r5 = Rm
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)helper, 0))// r0 = &helper
        return false;
    emit_jsr_at_rn(0);
    emit_nop();
    if (opc_writes_dest_s1(opc)) {
        emit_movl_rm_disp_rn(0, rd, 8);                   // arm_r.r[rd] = r0
    }
    return true;
}

// Phase 2 chunk 4: imm-shift forms. Common shape is a shifter-call
// preamble that leaves shifted Rm in r0, followed by the same body
// machinery the chunks-1/3 ops already use (re-keyed to read shifted
// Rm from r0 instead of a fresh memory load).
//
// Shifter preamble (5 SH4 insts + 1 literal):
//   mov.l @(rm, r8), r4         ; r4 = Rm
//   mov  #imm, r5                ; r5 = shift_imm (always in [1,31])
//   mov.l @(pc, lit), r0         ; r0 = &arm_jit_<sh>_v
//   jsr  @r0
//   nop                          ; delay slot; r0 := shifted_Rm on return

static void *arm_dp_reg_shifter(uint8_t shift_type, bool with_cout) {
    if (with_cout) {
        switch (shift_type) {
            case 0: return (void *)&arm_jit_lsl_vc;
            case 1: return (void *)&arm_jit_lsr_vc;
            case 2: return (void *)&arm_jit_asr_vc;
            case 3: return (void *)&arm_jit_ror_vc;
            default: return NULL;
        }
    }
    switch (shift_type) {
        case 0: return (void *)&arm_jit_lsl_v;
        case 1: return (void *)&arm_jit_lsr_v;
        case 2: return (void *)&arm_jit_asr_v;
        case 3: return (void *)&arm_jit_ror_v;
        default: return NULL;
    }
}

static bool emit_arm_shift_preamble(uint8_t rm, uint8_t shift_type,
                                    uint8_t shift_imm, bool with_cout) {
    void *helper = arm_dp_reg_shifter(shift_type, with_cout);
    if (!helper) return false;
    emit_movl_disp_rm_rn(rm, 8, 4);
    emit_mov_imm_rn((int8_t)shift_imm, 5);
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)helper, 0))
        return false;
    emit_jsr_at_rn(0);
    emit_nop();
    return true;
}

// Op-body sizes for kind 3 (S=0) after the shifter preamble (5 insts).
// r0 holds shifted Rm at entry to the body.
static int dp_reg_shifted_s0_body_size(uint8_t opc) {
    switch (opc) {
        case 0xD: return 1;                 // MOV: store r0 to Rd
        case 0xF: return 2;                 // MVN: not + store
        case 0xE: return 4;                 // BIC: load Rn, not, and, store
        default:  return 3;                 // AND/EOR/SUB/RSB/ADD/ORR
    }
}

// Op-body sizes for kind 4 (S=1 arith) after the shifter preamble.
//   r0 = shifted Rm at entry.
//   common: mov r0,r5; mov.l @(rn,r8),r4; mov.l @(pc,lit_helper),r0; jsr; nop
//   ADDS/SUBS add a final mov.l r0,@(rd,r8).
static int dp_reg_shifted_s1_body_size(uint8_t opc) {
    return opc_writes_dest_s1(opc) ? 6 : 5;
}

static int emit_arm_dp_reg_shifted_size(uint8_t opc, int kind) {
    // 5 SH4 insts for the shifter preamble plus the op-body.
    int op_body = (kind == 4) ? dp_reg_shifted_s1_body_size(opc)
                              : dp_reg_shifted_s0_body_size(opc);
    return 5 + op_body;
}

static bool emit_arm_dp_reg_shifted_body(uint8_t opc, uint8_t rn, uint8_t rd,
                                         uint8_t rm, uint8_t shift_type,
                                         uint8_t shift_imm, int kind) {
    // S=1 logic ops need the shifter cout staged into arm_flag_c so the
    // op helper (which only sets N/Z) leaves C reflecting the shifter.
    // S=0 ops and S=1 arith ops use the value-only shifter.
    bool with_cout = (kind == 4) && opc_s1_uses_shifter_cout(opc);
    if (!emit_arm_shift_preamble(rm, shift_type, shift_imm, with_cout)) return false;

    if (kind == 3) {
        // S=0 inline body. r0 = shifted Rm post-helper.
        switch (opc) {
            case 0xD:  // MOV Rd, Rm-shifted
                emit_movl_rm_disp_rn(0, rd, 8);
                return true;
            case 0xF:  // MVN Rd, Rm-shifted
                emit_not_rr(0, 0);
                emit_movl_rm_disp_rn(0, rd, 8);
                return true;
            default: break;
        }
        // 3-operand. Load Rn into r1, combine with shifted Rm in r0.
        emit_movl_disp_rm_rn(rn, 8, 1);    // r1 = Rn
        switch (opc) {
            case 0x0: emit_and_rr(1, 0); break;                 // AND
            case 0x1: emit_xor_rr(1, 0); break;                 // EOR
            case 0x2:                                            // SUB Rn - sh
                emit_sub_rr(0, 1);                               // r1 -= r0
                emit_movl_rm_disp_rn(1, rd, 8);
                return true;
            case 0x3:                                            // RSB sh - Rn
                emit_sub_rr(1, 0);                               // r0 -= r1
                break;
            case 0x4: emit_add_rr(1, 0); break;                 // ADD
            case 0xC: emit_or_rr (1, 0); break;                 // ORR
            case 0xE:                                            // BIC Rn & ~sh
                emit_not_rr(0, 0);                               // r0 = ~sh
                emit_and_rr(1, 0);                               // r0 &= r1
                break;
            default:
                return false;
        }
        emit_movl_rm_disp_rn(0, rd, 8);
        return true;
    }

    // kind == 4: S=1 arith helper call. shifted_Rm currently in r0;
    // move it to r5 (helper arg 2), load Rn into r4 (helper arg 1).
    void *helper = arm_dp_reg_s1_helper(opc);
    if (!helper) return false;
    emit_mov_rr(0, 5);                                  // r5 = shifted_Rm
    emit_movl_disp_rm_rn(rn, 8, 4);                     // r4 = Rn
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)helper, 0))
        return false;
    emit_jsr_at_rn(0);
    emit_nop();
    if (opc_writes_dest_s1(opc)) {
        emit_movl_rm_disp_rn(0, rd, 8);
    }
    return true;
}

// Recognition: returns one of
//   0 = no spec, fall through to JSR fallback
//   1 = chunk 1: no-shift, S=0, inline body
//   2 = chunk 3: no-shift, S=1 (TST/TEQ/CMP/CMN/ADDS/SUBS), helper call
//   3 = chunk 4: imm-shifted, S=0, inline body
//   4 = chunk 4: imm-shifted, S=1 arith (CMP/CMN/ADDS/SUBS), helper call
//
// All flavours share: group=000, imm-shift form (bit 4 = 0), Rd != 15.
// TST/TEQ with non-zero shift are deferred because their flag handling
// needs the shifter cout staged into arm_flag_c.
static int arm_dp_reg_kind(uint32_t op) {
    if (((op >> 25) & 0x7) != 0)  return 0;        // group 0b000
    if (((op >> 4)  & 0x1) != 0)  return 0;        // bit 4 = 0 -> imm-shift form
    if (((op >> 12) & 0xF) == 15) return 0;        // Rd != 15

    uint8_t opc        = (op >> 21) & 0xF;
    bool    s          = (op >> 20) & 0x1;
    uint8_t shift_imm  = (op >> 7)  & 0x1F;
    uint8_t shift_type = (op >> 5)  & 0x3;

    // Special-encoding shift forms (LSR #32, ASR #32, RRX) -- defer.
    if (shift_imm == 0 && shift_type != 0) return 0;

    bool shifted = (shift_imm != 0);

    if (!s) {
        // S=0: inline body. Chunk 1 (no-shift) or chunk 4 (shifted).
        switch (opc) {
            case 0x0: case 0x1: case 0x2: case 0x3: case 0x4:
            case 0xC: case 0xD: case 0xE: case 0xF:
                return shifted ? 3 : 1;
            default:
                return 0;
        }
    }
    // S=1: helper-call body. Chunk 3 (no-shift) or chunk 4 (shifted).
    // Chunk 5 extends this to S=1 logic ops; the shifted-body emitter
    // picks the cout-shifter for those so arm_flag_c reflects the
    // shifter output (logic-op semantics).
    //
    // Deferred: ADC/SBC/RSC (need C-flag input) and RSBS (arith with
    // swapped operands; helper not yet provided).
    switch (opc) {
        case 0x0:  // ANDS
        case 0x1:  // EORS
        case 0x2:  // SUBS
        case 0x4:  // ADDS
        case 0x8:  // TST
        case 0x9:  // TEQ
        case 0xA:  // CMP
        case 0xB:  // CMN
        case 0xC:  // ORRS
        case 0xD:  // MOVS
        case 0xE:  // BICS
        case 0xF:  // MVNS
            return shifted ? 4 : 2;
        default:
            return 0;
    }
}

// Phase 2 chunk 6: data-proc imm coverage. Group=001 ops route through
// arm_dec_call_legacy (or one of the four decoder-specialised handlers
// arm_dec_{add,sub,cmp,mov}_imm), all of which the JIT previously left
// as JSR fallbacks except MOV imm. This chunk inlines all of them by
// inspecting raw_op, exactly like chunks 1-5 did for data-proc REG.
//
// Decoder pre-rotates the 8-bit imm + 4-bit rot into uop->arg_c, so the
// JIT just needs (Rn, Rd, opc, S) from raw_op and the rotated value from
// arg_c. The cout for the imm rotation is bit 31 of the rotated value;
// it's known at JIT time and matters only for S=1 logic ops, where the
// JIT pre-writes arm_flag_c before calling the helper (which leaves C
// alone in the post-helper state).
//
// kind:
//   1 = S=0 inline body. Reuses the chunk-1 emit shape but with the imm
//       value loaded into r1 via the literal pool instead of from memory.
//   2 = S=1 arith helper-call (CMP/CMN/ADDS/SUBS imm). Same shape as
//       chunk 3 but with imm in r5 via literal pool.
//   3 = S=1 logic helper-call (TST/TEQ/ANDS/EORS/ORRS/BICS/MOVS/MVNS
//       imm). Extra 3-instruction "*arm_flag_c = cout" pre-set ahead
//       of the helper call.
//
// ADC/SBC/RSC imm are deferred (need C-flag input variants of the
// helpers, same gap as chunk 5 left for the REG forms).

static int arm_dp_imm_kind(uint32_t op) {
    if (((op >> 25) & 0x7) != 1)  return 0;        // group 001
    if (((op >> 12) & 0xF) == 15) return 0;        // Rd != 15

    uint8_t opc = (op >> 21) & 0xF;
    bool    s   = (op >> 20) & 0x1;

    // Chunk 1 already inlines MOV imm S=0; leave that alone.
    if (opc == 0xD && !s) return 0;

    if (!s) {
        switch (opc) {
            case 0x0: case 0x1: case 0x2: case 0x3: case 0x4:
            case 0xC: case 0xE: case 0xF:
                return 1;
            default:
                return 0;  // ADC/SBC/RSC S=0 deferred
        }
    }
    switch (opc) {
        case 0x2:  // SUBS imm
        case 0x4:  // ADDS imm
        case 0xA:  // CMP imm
        case 0xB:  // CMN imm
            return 2;
        case 0x0:  // ANDS imm
        case 0x1:  // EORS imm
        case 0x8:  // TST  imm
        case 0x9:  // TEQ  imm
        case 0xC:  // ORRS imm
        case 0xD:  // MOVS imm
        case 0xE:  // BICS imm
        case 0xF:  // MVNS imm
            return 3;
        default:
            return 0;  // ADC/SBC/RSC S=1 deferred
    }
}

// Body sizes for the bt-skip displacement of the cond-checked variant.
// Sizes count SH4 instructions in the body (no prefix, no bt).
static int emit_arm_dp_imm_size(uint8_t opc, int kind) {
    if (kind == 1) {
        switch (opc) {
            case 0xF: return 3;                 // MVN imm: load ~imm, store
            case 0xE: return 5;                 // BIC: load Rn, load imm, not, and, store
            default:  return 4;                 // AND/EOR/SUB/RSB/ADD/ORR imm
        }
    }
    if (kind == 2) {
        // CMP/CMN no store; ADDS/SUBS add a final mov.l r0, @(rd, r8).
        return opc_writes_dest_s1(opc) ? 6 : 5;
    }
    // kind 3: 3-inst cout pre-set + helper call (5) + optional store.
    return 3 + (opc_writes_dest_s1(opc) ? 6 : 5);
}

// kind 1: S=0 inline body. r0 = Rn (or unused for MVN), r1 = imm.
static bool emit_arm_dp_imm_s0_body(uint8_t opc, uint8_t rn, uint8_t rd, uint32_t imm) {
    if (opc == 0xF) {                              // MVN imm: Rd = ~imm
        if (!emit_movl_pc_lit(~imm, 0)) return false;
        emit_movl_rm_disp_rn(0, rd, 8);
        return true;
    }
    emit_movl_disp_rm_rn(rn, 8, 0);                // r0 = Rn
    if (!emit_movl_pc_lit(imm, 1)) return false;   // r1 = imm
    switch (opc) {
        case 0x0: emit_and_rr(1, 0); break;        // AND
        case 0x1: emit_xor_rr(1, 0); break;        // EOR
        case 0x2: emit_sub_rr(1, 0); break;        // SUB
        case 0x3:                                   // RSB: imm - Rn
            emit_sub_rr(0, 1);                      // r1 -= r0
            emit_movl_rm_disp_rn(1, rd, 8);
            return true;
        case 0x4: emit_add_rr(1, 0); break;        // ADD
        case 0xC: emit_or_rr (1, 0); break;        // ORR
        case 0xE:                                   // BIC: Rn & ~imm
            emit_not_rr(1, 1);                      // r1 = ~imm
            emit_and_rr(1, 0);                      // r0 &= r1
            break;
        default: return false;
    }
    emit_movl_rm_disp_rn(0, rd, 8);
    return true;
}

// kind 2: S=1 arith helper-call. r4 = Rn, r5 = imm.
static bool emit_arm_dp_imm_s1_arith_body(uint8_t opc, uint8_t rn, uint8_t rd, uint32_t imm) {
    void *helper = arm_dp_reg_s1_helper(opc);
    if (!helper) return false;
    emit_movl_disp_rm_rn(rn, 8, 4);                            // r4 = Rn
    if (!emit_movl_pc_lit(imm, 5)) return false;               // r5 = imm
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)helper, 0))     // r0 = &helper
        return false;
    emit_jsr_at_rn(0);
    emit_nop();
    if (opc_writes_dest_s1(opc)) {
        emit_movl_rm_disp_rn(0, rd, 8);
    }
    return true;
}

// kind 3: S=1 logic helper-call with cout pre-set.
//   r1 = &arm_flag_c ; r0 = cout (constant 0 or 1) ; mov.l r0, @r1
// followed by the same shape as kind 2.
static bool emit_arm_dp_imm_s1_logic_body(uint8_t opc, uint8_t rn, uint8_t rd, uint32_t imm) {
    void *helper = arm_dp_reg_s1_helper(opc);
    if (!helper) return false;
    uint8_t cout = (uint8_t)((imm >> 31) & 1u);

    // Stash cout into arm_flag_c so the helper (which doesn't touch C)
    // leaves the right state for subsequent ops.
    emit_mov_imm_rn((int8_t)cout, 0);
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)&arm_flag_c, 1)) return false;
    emit_movl_rm_disp_rn(0, 0, 1);                              // mov.l r0, @r1

    emit_movl_disp_rm_rn(rn, 8, 4);                            // r4 = Rn
    if (!emit_movl_pc_lit(imm, 5)) return false;               // r5 = imm
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)helper, 0))     // r0 = &helper
        return false;
    emit_jsr_at_rn(0);
    emit_nop();
    if (opc_writes_dest_s1(opc)) {
        emit_movl_rm_disp_rn(0, rd, 8);
    }
    return true;
}

static bool emit_arm_dp_imm(uint8_t cond, uint32_t op, int kind, uint32_t imm) {
    uint8_t opc = (op >> 21) & 0xF;
    uint8_t rn  = (op >> 16) & 0xF;
    uint8_t rd  = (op >> 12) & 0xF;

    int body_size = emit_arm_dp_imm_size(opc, kind);

    if (cond != 14) {
        if (!emit_arm_cond_check_prefix(cond)) return false;
        emit_bt((int16_t)(body_size - 1));
    }
    switch (kind) {
        case 1: return emit_arm_dp_imm_s0_body       (opc, rn, rd, imm);
        case 2: return emit_arm_dp_imm_s1_arith_body (opc, rn, rd, imm);
        case 3: return emit_arm_dp_imm_s1_logic_body (opc, rn, rd, imm);
        default: return false;
    }
}

// Dispatch one uop. Returns SPECIALISED, FALLBACK_OK, or FAIL.
enum emit_result { EMIT_SPECIALISED, EMIT_FALLBACK, EMIT_FAIL };

static enum emit_result arm_emit_one_uop(const arm_uop_t *uop) {
    // Decoder UNCOND (15) is only valid for legacy fallback.
    if (uop->cond == 15) {
        if (!emit_jsr_fallback(uop)) return EMIT_FAIL;
        return EMIT_FALLBACK;
    }
    if (uop->handler == arm_dec_b_imm24) {
        if (!emit_arm_b_imm24(uop->cond, (int32_t)uop->arg_c)) return EMIT_FAIL;
        return EMIT_SPECIALISED;
    }
    if (uop->handler == arm_dec_bl_imm24) {
        if (!emit_arm_bl_imm24(uop->cond, (int32_t)uop->arg_c)) return EMIT_FAIL;
        return EMIT_SPECIALISED;
    }
    if (uop->handler == arm_dec_mov_imm) {
        if (!emit_arm_mov_imm(uop->cond, uop->arg_b, uop->arg_c)) return EMIT_FAIL;
        return EMIT_SPECIALISED;
    }
    // Phase 2 chunk 6: data-proc imm. arm_dp_imm_kind keys on raw_op so
    // it catches both the unspecialised legacy ops (handler ==
    // arm_dec_call_legacy) and the decoder-specialised handlers (ADD/SUB
    // /CMP imm) we previously left as JSR fallbacks. The pre-rotated
    // 32-bit imm lives in uop->arg_c.
    {
        int kind = arm_dp_imm_kind(uop->raw_op);
        if (kind) {
            if (!emit_arm_dp_imm(uop->cond, uop->raw_op, kind, uop->arg_c))
                return EMIT_FAIL;
            return EMIT_SPECIALISED;
        }
    }
    // Phase 2 chunks 1 & 3: JIT-only recognition of data-proc REG
    // (no-shift). The interpreter routes these through
    // arm_dec_call_legacy; we intercept on the JIT side by inspecting
    // raw_op. arm_dp_reg_kind returns 1 for the chunk-1 S=0 inline-body
    // subset and 2 for the chunk-3 S=1 flag-update helper-call subset.
    if (uop->handler == arm_dec_call_legacy) {
        int kind = arm_dp_reg_kind(uop->raw_op);
        if (kind) {
            if (!emit_arm_dp_reg(uop->cond, uop->raw_op, kind)) return EMIT_FAIL;
            return EMIT_SPECIALISED;
        }
    }
    if (!emit_jsr_fallback(uop)) return EMIT_FAIL;
    return EMIT_FALLBACK;
}

bool arm_jit_compile_block(arm_block_t *b) {
    if (!jit_enabled)                  return false;
    if (b->length == 0)                return false;
    if (b->length > ARM_BLOCK_MAX_LEN) return false;

    // Chunk 7: RAM-resident ARM blocks are now JIT candidates again.
    // Many GBA carts copy ARM hot-paths into IWRAM; with finer-grained
    // page-gen tracking (see thumb_block.c) those stable copies now
    // survive the cart's data writes and stay JIT'd.

    size_t budget = estimate_block_bytes(b->length);
    uint16_t *entry = jit_emit_begin(budget);
    if (!entry) return false;

    emit_movl_pc_lit_reset();

    const arm_uop_t *ops = arm_uop_pool + b->ops_offset;

    // -------- Prologue --------
    emit_pushl_rr(8, 15);          // mov.l r8, @-r15
    emit_sts_l_pr(15);             // sts.l pr, @-r15
    emit_mov_rr(4, 8);             // mov r4, r8       ; r8 = &arm_r

    // -------- Per-uop body (all but the last) --------
    // ARM steps R15 by 4 per instruction.
    uint32_t specialised = 0;
    for (uint16_t i = 0; i + 1u < b->length; i++) {
        enum emit_result r = arm_emit_one_uop(&ops[i]);
        if (r == EMIT_FAIL) goto fail;
        if (r == EMIT_SPECIALISED) specialised++;
        emit_arm_r15_add(4);
    }

    // -------- Last uop --------
    {
        uint16_t last = (uint16_t)(b->length - 1u);
        enum emit_result r = arm_emit_one_uop(&ops[last]);
        if (r == EMIT_FAIL) goto fail;
        if (r == EMIT_SPECIALISED) specialised++;
    }

    // -------- Post-block: branch-taken check --------
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)&pipe_reload, 0)) goto fail;
    emit_movb_at_rm_rn(0, 1);
    emit_tst_rr(1, 1);
    // bf to epilogue: if T=0 (pipe_reload != 0), skip sequential exit.
    // Sequential-exit block below is 6 instructions; bf disp_insts=5.
    emit_bf(5);

    // -------- Sequential exit (pipe_reload was 0) --------
    // Interp: R15 += 4 (last-uop post-inc the per-uop loop skipped) then
    // R15 -= 8 (seq exit). Collapsed to R15 -= 4 here.
    emit_arm_r15_add(-4);
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)&arm_load_pipe, 0)) goto fail;
    emit_jsr_at_rn(0);
    emit_nop();

    // -------- Epilogue --------
    emit_lds_l_pr(15);              // lds.l @r15+, pr
    emit_popl_rr(15, 8);            // mov.l @r15+, r8
    emit_rts();
    emit_nop();

    // -------- Literal pool --------
    if (!emit_literal_pool_finalize()) goto fail;

    b->native_entry = jit_emit_end(entry);

#if GBA_BENCH
    bench_arm_jit_compiled++;
    bench_arm_jit_specialized_ops += specialised;
#else
    (void)specialised;
#endif

    return true;

fail:
    jit_rewind(entry);
    return false;
}
