#include "thumb_jit.h"

#include <stdint.h>
#include <stddef.h>

#include "arm.h"            // pipe_reload, arm_load_pipe
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

extern void t16_dec_mov_imm8(const thumb_uop_t *uop);

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

// Dispatch one uop. Returns SPECIALISED, FALLBACK_OK, or FAIL.
enum emit_result { EMIT_SPECIALISED, EMIT_FALLBACK, EMIT_FAIL };

static enum emit_result emit_one_uop(const thumb_uop_t *uop) {
    if (uop->handler == t16_dec_mov_imm8) {
        if (!emit_thumb_mov_imm8(uop->arg_a, uop->arg_b)) return EMIT_FAIL;
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
