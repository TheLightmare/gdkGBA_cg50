#include "thumb_jit.h"

#include <stdint.h>
#include <stddef.h>

#include "arm.h"            // pipe_reload, arm_load_pipe
#include "sh4_emit.h"
#include "sh4_jit.h"
#include "thumb_block.h"

// Phase 1 Chunk 1: every uop in the block becomes a JSR to its existing
// C handler. No opcode specialisation yet. The point is to validate
// prologue, calling convention, literal-pool fix-up, sequential vs.
// branch exit, and decoder-side install before any per-opcode codegen.
//
// Register usage in emitted code:
//   r0    scratch (literal-pool target; handler call target)
//   r1    scratch (pipe_reload byte)
//   r4    first arg to legacy handler (uop ptr); reloaded per uop
//   r8    &arm_r for the lifetime of the block; saved on entry
//   r15   stack pointer
//   PR    saved on entry, restored on exit
//
// Calling convention out: per the convention in sh4_jit.h, the executor
// pre-credits b->total_cycles before invoking us and clears pipe_reload
// before and after the call. The native block must leave arm_r in the
// same final state the interpreter would, which means either:
//   - the last handler called arm_load_pipe() itself (branch taken),
//     leaving pipe_reload != 0 and R15 already updated; or
//   - we sequential-exit by doing R15 -= 2 and calling arm_load_pipe()
//     (matching the interpreter's R15+=2 (last-iter post-increment) plus
//     R15-=4 (sequential exit) collapsed into one).
//
// Decoder guarantee: t16_block_ends terminates a block at the first
// PC-write opcode, so only the LAST uop can plausibly set pipe_reload.
// We therefore skip the per-uop pipe_reload check and do one check at
// block end. Exotic mid-block exceptions (e.g. an UND in the middle of
// a block) would not be caught by this -- they are out of Phase 1
// scope and addressed if they show up empirically.

// arm_r layout: r[16] starts at offset 0. r[15] lives at byte offset 60,
// which is long-disp 15 -- exactly fitting the 4-bit disp field of
// `mov.l @(disp,Rm),Rn`.
#define ARM_R15_LONG_DISP 15

// Budget bound for a block of `length` uops. Used for the arena
// reservation in jit_emit_begin(). The real footprint is smaller (see
// the per-section breakdown in the code below); we add slack so the
// reservation never under-counts.
static size_t estimate_block_bytes(uint16_t length) {
    // Code (worst case, per uop = 7 inst = 14 B; last uop = 4 inst = 8 B):
    //   prologue (6 B) + 14*(length-1) + 8 + post-block (~28 B)
    // Literal pool (2 longs per uop + 2 globals = 2*length + 2 longs):
    //   (2*length + 2) * 4 bytes, plus up to 2 B of nop alignment.
    size_t code = 6u + 14u * (size_t)length + 36u;
    size_t lits = (2u * (size_t)length + 2u) * 4u + 2u;
    return code + lits + 16u;   // a little extra slack
}

bool thumb_jit_compile_block(thumb_block_t *b) {
    if (!jit_enabled)                    return false;
    if (b->length == 0)                  return false;
    if (b->length > THUMB_BLOCK_MAX_LEN) return false;

    size_t budget = estimate_block_bytes(b->length);
    uint16_t *entry = jit_emit_begin(budget);
    if (!entry) return false;

    emit_movl_pc_lit_reset();

    const thumb_uop_t *ops = thumb_uop_pool + b->ops_offset;

    // -------- Prologue --------
    emit_pushl_rr(8, 15);          // mov.l r8, @-r15
    emit_sts_l_pr(15);             // sts.l pr, @-r15
    emit_mov_rr(4, 8);             // mov r4, r8       ; r8 = &arm_r

    // -------- Per-uop body (all but the last) --------
    // Each uop: load &uop into r4, load handler into r0, jsr;nop,
    // then arm_r.r[15] += 2 so the next handler sees the right PC.
    for (uint16_t i = 0; i + 1u < b->length; i++) {
        if (!emit_movl_pc_lit((uint32_t)(uintptr_t)&ops[i],         4)) goto fail;
        if (!emit_movl_pc_lit((uint32_t)(uintptr_t)ops[i].handler,  0)) goto fail;
        emit_jsr_at_rn(0);
        emit_nop();
        // r0 = arm_r.r[15]; r0 += 2; arm_r.r[15] = r0
        emit_movl_disp_rm_rn(ARM_R15_LONG_DISP, 8, 0);
        emit_add_imm_rn(2, 0);
        emit_movl_rm_disp_rn(0, ARM_R15_LONG_DISP, 8);
    }

    // -------- Last uop (no post-increment; the post-block check
    //         decides whether to step R15 or rely on the handler's
    //         arm_load_pipe()) --------
    uint16_t last = (uint16_t)(b->length - 1u);
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)&ops[last],        4)) goto fail;
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)ops[last].handler, 0)) goto fail;
    emit_jsr_at_rn(0);
    emit_nop();

    // -------- Post-block: branch-taken check --------
    // r0 = &pipe_reload; r1 = signed-byte *r0; tst r1,r1 -> T = (r1==0).
    // pipe_reload is C bool (0 or 1) -- no extu.b needed, signed load
    // still tests non-zero correctly via the AND-with-self.
    if (!emit_movl_pc_lit((uint32_t)(uintptr_t)&pipe_reload, 0)) goto fail;
    emit_movb_at_rm_rn(0, 1);
    emit_tst_rr(1, 1);

    // bf to epilogue. If T=0 (pipe_reload != 0, i.e. branch taken), skip
    // the sequential-exit fix-ups -- the handler already updated R15
    // and called arm_load_pipe(). disp_insts=5 lands at the first
    // epilogue instruction (6 instructions of sequential exit between).
    emit_bf(5);

    // -------- Sequential exit (pipe_reload was 0) --------
    // Interpreter: increments R15 += 2 after the last handler, then
    // sequential exit does R15 -= 4 and arm_load_pipe(). Collapsed to
    // R15 -= 2 here.
    emit_movl_disp_rm_rn(ARM_R15_LONG_DISP, 8, 0);
    emit_add_imm_rn(-2, 0);
    emit_movl_rm_disp_rn(0, ARM_R15_LONG_DISP, 8);
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
    return true;

fail:
    jit_rewind(entry);
    return false;
}
