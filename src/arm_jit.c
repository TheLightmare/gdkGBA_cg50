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

static size_t estimate_block_bytes(uint16_t length) {
    // Worst case per uop ~22 B (JSR fallback inc. R15 += 4). Last uop
    // ~10 B. Post-block + epilogue ~28 B. Literal pool: 2 longs per uop
    // for the JSR-fallback path + 2 globals.
    size_t code = 6u + 22u * (size_t)length + 36u;
    size_t lits = (2u * (size_t)length + 2u) * 4u + 2u;
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
    for (uint16_t i = 0; i + 1u < b->length; i++) {
        if (!emit_jsr_fallback(&ops[i])) goto fail;
        emit_arm_r15_add(4);
    }

    // -------- Last uop --------
    {
        uint16_t last = (uint16_t)(b->length - 1u);
        if (!emit_jsr_fallback(&ops[last])) goto fail;
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
#endif

    return true;

fail:
    jit_rewind(entry);
    return false;
}
