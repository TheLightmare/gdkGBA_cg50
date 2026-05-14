#include "thumb_jit.h"

#include <stdint.h>
#include <stddef.h>

#include "arm.h"            // pipe_reload, arm_load_pipe
#include "arm_mem.h"        // arm_read, arm_write
#include "bench.h"
#include "build_flags.h"
#include "sh4_emit.h"
#include "sh4_jit.h"
#include "thumb_block.h"

// Phase 1 JIT: each Thumb block becomes an SH4 routine. Uops covered by
// a specialised emitter run as inline native code; everything else
// falls back to a JSR to the existing C handler. Phase 1 specialisation
// roster lives in try_specialize() below.
//
// Register usage in emitted code:
//   r0    scratch (literal targets, value temps, handler call target)
//   r1    scratch (pipe_reload byte, flag-address pointer)
//   r4    first arg to legacy handler (uop ptr); reloaded per uop
//   r8    &arm_r for the lifetime of the block; saved on entry
//   r15   stack pointer
//   PR    saved on entry, restored on exit
//
// Calling convention out: per sh4_jit.h, the executor pre-credits
// b->total_cycles and clears pipe_reload before invoking us. The native
// block must leave arm_r in the same final state the interpreter would.
// Decoder guarantee: t16_block_ends() terminates a block at the first
// PC-write opcode, so only the LAST uop can plausibly set pipe_reload.
// We therefore skip the per-uop pipe_reload check and do one check at
// block end. Specialised non-branch ops never touch pipe_reload, so
// when the LAST uop is specialised, the post-block check falls through
// to the sequential exit branch -- correct.

// arm_r layout: r[16] starts at offset 0. r[Rd] is at byte offset Rd*4,
// long-disp = Rd, which fits the 4-bit disp field of mov.l @(d,Rm),Rn
// for Rd in 0..15.

// ----- Externs reached by specialisations ----------------------------------
//
// Declared here rather than added to arm.h to keep the public interface
// stable. These are private globals the JIT pokes at directly.
extern uint32_t arm_flag_n;
extern uint32_t arm_flag_z;

extern bool int_halt;

extern void t16_dec_mov_imm8(const thumb_uop_t *uop);
extern void t16_dec_b_imm11(const thumb_uop_t *uop);
extern void t16_dec_b_cond (const thumb_uop_t *uop);
extern void t16_dec_ldr_imm5(const thumb_uop_t *uop);
extern void t16_dec_str_imm5(const thumb_uop_t *uop);
extern void t16_dec_ldr_pc8 (const thumb_uop_t *uop);

// ----- Internal helpers ----------------------------------------------------

static size_t estimate_block_bytes(uint16_t length) {
    // Worst case per uop ~22 B (largest of the per-op patterns we
    // emit today: JSR fallback inc. R15+=2). Last uop ~10 B. Post-
    // block + epilogue ~28 B. Literals: a specialised emit may need
    // up to 4 literals per uop (op value + flag_n addr + flag_z addr
    // + ...) so size for the larger of (2/uop for fallback,
    // up-to-4/uop for spec) to keep the bound conservative.
    size_t code = 6u + 22u * (size_t)length + 36u;
    size_t lits = (4u * (size_t)length + 2u) * 4u + 2u;
    return code + lits + 32u;
}

// JSR-to-legacy-handler pattern. Returns false iff the literal pool ran
// out (the codegen caller bails and rewinds).
static bool emit_jsr_fallback(const thumb_uop_t *uop) {
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)uop,          4)) return false;
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)uop->handler, 0)) return false;
    emit_jsr_at_rn(0);
    emit_nop();
    return true;
}

// r0 = arm_r.r[15]; r0 += delta; arm_r.r[15] = r0. delta fits the
// SH4 add #imm 8-bit signed encoding.
static void emit_arm_r15_add(int8_t delta) {
    emit_movl_disp_rm_rn(15, 8, 0);   // r0 = arm_r.r[15]
    emit_add_imm_rn(delta, 0);        // r0 += delta
    emit_movl_rm_disp_rn(0, 15, 8);   // arm_r.r[15] = r0
}

// ----- Specialised emitters ------------------------------------------------

// Thumb MOV Rd, #imm8 (arg_a = Rd in 0..7, arg_b = imm8 in 0..255).
//   arm_r.r[Rd] = imm8
//   arm_flag_n  = 0                    (imm8 <= 0xFF -> bit 31 is 0)
//   arm_flag_z  = (imm8 == 0) ? 1 : 0  (constant at codegen time)
// 7 SH4 instructions + 3 literal-pool entries.
static bool emit_thumb_mov_imm8(uint8_t rd, uint8_t imm8) {
    // r0 = imm8  (use literal pool: imm8 may exceed mov #imm's signed 8b range)
    if (!emit_movl_pc_lit((uint32_t)imm8, 0)) return false;
    // arm_r.r[Rd] = r0
    emit_movl_rm_disp_rn(0, (uint8_t)(rd & 0xF), 8);

    // arm_flag_n = 0
    emit_mov_imm_rn(0, 0);                                       // r0 = 0
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)&arm_flag_n, 1))  // r1 = &arm_flag_n
        return false;
    emit_movl_rm_disp_rn(0, 0, 1);                               // *r1 = r0

    // arm_flag_z = (imm8 == 0) ? 1 : 0
    emit_mov_imm_rn(imm8 == 0 ? 1 : 0, 0);                       // r0 = z value
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)&arm_flag_z, 1))  // r1 = &arm_flag_z
        return false;
    emit_movl_rm_disp_rn(0, 0, 1);                               // *r1 = r0

    return true;
}

// Thumb B #imm11 (arg_c = sign-extended imm11<<1 in int16). Block-ending.
//   arm_r.r[15] += imm
//   if (imm == -4) int_halt = true       (idle-spin: `b .` waits for IRQ)
//   arm_load_pipe()
// The handler returns having set pipe_reload=true (via arm_load_pipe),
// so the post-block check correctly takes the skip-sequential-exit
// branch. ~7-10 SH4 instructions + 2-3 literals.
static bool emit_thumb_b_imm11(int32_t imm) {
    // r0 = arm_r.r[15]
    emit_movl_disp_rm_rn(15, 8, 0);
    // r0 += imm. imm range is [-2048, +2046] (imm11<<1). SH4 add #imm
    // sign-extends an 8-bit imm; if imm fits int8 we can skip the
    // literal-pool load.
    if (imm >= -128 && imm <= 127) {
        emit_add_imm_rn((int8_t)imm, 0);
    } else {
        // r1 = imm (32-bit literal); r0 += r1
        if (!emit_movl_pc_lit((uint32_t)imm, 1)) return false;
        emit_add_rr(1, 0);
    }
    // arm_r.r[15] = r0
    emit_movl_rm_disp_rn(0, 15, 8);

    // The `b .` idle spin: hand off to arm_exec's int_halt path so we
    // don't burn cycles re-entering the same block until target_cycles.
    if (imm == -4) {
        emit_mov_imm_rn(1, 0);                                          // r0 = 1
        if (!emit_movl_pc_lit((uint32_t)(uintptr_t)&int_halt, 1))       // r1 = &int_halt
            return false;
        emit_movb_rm_at_rn(0, 1);                                       // *r1 = (byte)r0
    }

    // arm_load_pipe(); sets pipe_reload = true.
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)&arm_load_pipe, 0)) return false;
    emit_jsr_at_rn(0);
    emit_nop();
    return true;
}

// Thumb B<cond> #imm8 (arg_a = cond 0..13, arg_c = sign-extended imm8<<1
// in int16, range [-256, +254]). Block-ending. Always fits SH4 add #imm.
//   if (arm_cond(cond)) { arm_r.r[15] += imm; arm_load_pipe(); }
// Codegen: set r4 = cond, JSR arm_cond_check, then bt over the branch
// body if r0 == 0 (cond false). When cond is true we run the same
// branch-take sequence as B imm11. ~13 SH4 instructions + 3 literals.
//
// Conds 14 and 15 are decoded as UDF/SWI (decoder routes them through
// the legacy fallback), so we never see them here -- but we still
// guard defensively.
static bool emit_thumb_b_cond(uint8_t cond, int32_t imm) {
    if (cond >= 14) return false;     // decoder shouldn't emit this
    if (imm < -128 || imm > 127) return false;   // out of cond-B range

    // r4 = cond
    emit_mov_imm_rn((int8_t)cond, 4);
    // r0 = &arm_cond_check
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)&arm_cond_check, 0))
        return false;
    // jsr @r0 ; nop          (r0 := bool result on return)
    emit_jsr_at_rn(0);
    emit_nop();
    // tst r0, r0             (T = (r0 == 0) => cond was false)
    emit_tst_rr(0, 0);
    // bt over the branch-take sequence. 6 instructions follow; target
    // is the instruction immediately after them, so disp_insts = 5
    // (bt target = bt_pc + 4 + 2*disp, lands at +14 = 7th-slot offset).
    emit_bt(5);

    // r15 += imm (imm fits int8 for cond-B)
    emit_movl_disp_rm_rn(15, 8, 0);
    emit_add_imm_rn((int8_t)imm, 0);
    emit_movl_rm_disp_rn(0, 15, 8);

    // arm_load_pipe()
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)&arm_load_pipe, 0)) return false;
    emit_jsr_at_rn(0);
    emit_nop();
    return true;
}

// Thumb LDR Rd, [Rn, #imm5*4] (arg_a = Rd, arg_b = Rn, arg_c = imm5*4).
// imm5*4 maxes at 124, fits SH4 add #imm. arm_read returns the loaded
// word in r0. 7 SH4 instructions + 1 literal.
static bool emit_thumb_ldr_imm5(uint8_t rd, uint8_t rn, uint16_t imm) {
    emit_movl_disp_rm_rn((uint8_t)(rn & 0xF), 8, 0);        // r0 = arm_r.r[Rn]
    emit_add_imm_rn((int8_t)imm, 0);                         // r0 += imm
    emit_mov_rr(0, 4);                                       // r4 = r0 (address)
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)&arm_read, 0)) return false;
    emit_jsr_at_rn(0);
    emit_nop();
    emit_movl_rm_disp_rn(0, (uint8_t)(rd & 0xF), 8);         // arm_r.r[Rd] = r0
    return true;
}

// Thumb STR Rd, [Rn, #imm5*4] (arg_a = Rt, arg_b = Rn, arg_c = imm5*4).
// arm_write takes (addr, value) in r4/r5; no return value. 7 SH4
// instructions + 1 literal.
static bool emit_thumb_str_imm5(uint8_t rt, uint8_t rn, uint16_t imm) {
    emit_movl_disp_rm_rn((uint8_t)(rn & 0xF), 8, 0);        // r0 = arm_r.r[Rn]
    emit_add_imm_rn((int8_t)imm, 0);                         // r0 += imm
    emit_mov_rr(0, 4);                                       // r4 = r0 (address)
    emit_movl_disp_rm_rn((uint8_t)(rt & 0xF), 8, 5);        // r5 = arm_r.r[Rt]
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)&arm_write, 0)) return false;
    emit_jsr_at_rn(0);
    emit_nop();
    return true;
}

// Thumb LDR Rd, [PC, #imm8*4] (arg_a = Rd, arg_c = imm8*4 in 0..1020).
// Address = (arm_r.r[15] & ~3) + imm. R15 here is the interpreter's
// usual "inst_pc + 4" -- the JIT increments R15 between uops, so by the
// time this body runs, R15 already reflects the right per-instruction
// value. 9-11 SH4 instructions + 1-2 literals depending on imm range.
static bool emit_thumb_ldr_pc8(uint8_t rd, uint16_t imm) {
    emit_movl_disp_rm_rn(15, 8, 0);     // r0 = arm_r.r[15]
    emit_shlr2(0);                      // clear low 2 bits via >>2 then <<2
    emit_shll2(0);
    if (imm <= 127u) {
        emit_add_imm_rn((int8_t)imm, 0);                     // r0 += imm
    } else {
        if (!emit_movl_pc_lit((uint32_t)imm, 1)) return false;
        emit_add_rr(1, 0);                                   // r0 += r1
    }
    emit_mov_rr(0, 4);                                       // r4 = r0 (address)
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)&arm_read, 0)) return false;
    emit_jsr_at_rn(0);
    emit_nop();
    emit_movl_rm_disp_rn(0, (uint8_t)(rd & 0xF), 8);         // arm_r.r[Rd] = r0
    return true;
}

// Dispatch one uop. Returns SPECIALISED, FALLBACK_OK, or FAIL.
enum emit_result { EMIT_SPECIALISED, EMIT_FALLBACK, EMIT_FAIL };

static enum emit_result emit_one_uop(const thumb_uop_t *uop) {
    if (uop->handler == t16_dec_mov_imm8) {
        if (!emit_thumb_mov_imm8(uop->arg_a, uop->arg_b)) return EMIT_FAIL;
        return EMIT_SPECIALISED;
    }
    if (uop->handler == t16_dec_b_imm11) {
        if (!emit_thumb_b_imm11((int32_t)(int16_t)uop->arg_c)) return EMIT_FAIL;
        return EMIT_SPECIALISED;
    }
    if (uop->handler == t16_dec_b_cond) {
        if (!emit_thumb_b_cond(uop->arg_a, (int32_t)(int16_t)uop->arg_c)) {
            // Spec rejected (out-of-range imm, cond>=14, lit overflow);
            // fall through to JSR fallback so the block still runs.
            if (!emit_jsr_fallback(uop)) return EMIT_FAIL;
            return EMIT_FALLBACK;
        }
        return EMIT_SPECIALISED;
    }
    if (uop->handler == t16_dec_ldr_imm5) {
        if (!emit_thumb_ldr_imm5(uop->arg_a, uop->arg_b, uop->arg_c)) return EMIT_FAIL;
        return EMIT_SPECIALISED;
    }
    if (uop->handler == t16_dec_str_imm5) {
        if (!emit_thumb_str_imm5(uop->arg_a, uop->arg_b, uop->arg_c)) return EMIT_FAIL;
        return EMIT_SPECIALISED;
    }
    if (uop->handler == t16_dec_ldr_pc8) {
        if (!emit_thumb_ldr_pc8(uop->arg_a, uop->arg_c)) return EMIT_FAIL;
        return EMIT_SPECIALISED;
    }
    if (!emit_jsr_fallback(uop)) return EMIT_FAIL;
    return EMIT_FALLBACK;
}

// ----- Compiler entry point ------------------------------------------------

bool thumb_jit_compile_block(thumb_block_t *b) {
    if (!jit_enabled)                    return false;
    if (b->length == 0)                  return false;
    if (b->length > THUMB_BLOCK_MAX_LEN) return false;

    // Chunk 7: RAM-resident blocks are now JIT candidates again. The
    // finer page-gen granularity (64-byte pages, down from 256) means
    // a data write to a page-co-tenant variable no longer invalidates
    // unrelated code -- stable copy-to-IWRAM code can survive the
    // cart's normal data churn and stay JIT'd.

    size_t budget = estimate_block_bytes(b->length);
    uint16_t *entry = jit_emit_begin(budget);
    if (!entry) return false;

    emit_movl_pc_lit_reset();

    const thumb_uop_t *ops = thumb_uop_pool + b->ops_offset;
    uint32_t specialised = 0;

    // -------- Prologue --------
    emit_pushl_rr(8, 15);          // mov.l r8, @-r15
    emit_sts_l_pr(15);             // sts.l pr, @-r15
    emit_mov_rr(4, 8);             // mov r4, r8       ; r8 = &arm_r

    // -------- Per-uop body (all but the last) --------
    for (uint16_t i = 0; i + 1u < b->length; i++) {
        enum emit_result r = emit_one_uop(&ops[i]);
        if (r == EMIT_FAIL) goto fail;
        if (r == EMIT_SPECIALISED) specialised++;
        emit_arm_r15_add(2);
    }

    // -------- Last uop (no post-increment; post-block check below
    //         resolves sequential vs. branch-taken exit) --------
    {
        uint16_t last = (uint16_t)(b->length - 1u);
        enum emit_result r = emit_one_uop(&ops[last]);
        if (r == EMIT_FAIL) goto fail;
        if (r == EMIT_SPECIALISED) specialised++;
    }

    // -------- Post-block: branch-taken check --------
    // r0 = &pipe_reload; r1 = signed-byte *r0; tst r1,r1 -> T = (r1==0).
    // pipe_reload is C bool (0 or 1) -- no extu.b needed.
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)&pipe_reload, 0)) goto fail;
    emit_movb_at_rm_rn(0, 1);
    emit_tst_rr(1, 1);
    // bf to epilogue: if T=0 (pipe_reload != 0), branch was taken, skip
    // sequential-exit fix-ups. disp_insts=5 lands at the first epilogue
    // instruction (6 instructions of sequential exit between).
    emit_bf(5);

    // -------- Sequential exit (pipe_reload was 0) --------
    // Collapsed (interp R15+=2 post-handler) + (R15-=4 seq-exit) into
    // R15 -= 2 here.
    emit_arm_r15_add(-2);
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
    bench_thumb_jit_compiled++;
    bench_thumb_jit_specialized_ops += specialised;
#else
    (void)specialised;
#endif

    return true;

fail:
    jit_rewind(entry);
    return false;
}
