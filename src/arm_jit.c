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
extern void arm_dec_b_imm24 (const arm_uop_t *uop);
extern void arm_dec_bl_imm24(const arm_uop_t *uop);
extern void arm_dec_mov_imm (const arm_uop_t *uop);

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
