#ifndef THUMB_BLOCK_H
#define THUMB_BLOCK_H

#include <stdint.h>
#include <stdbool.h>

#include "sh4_jit.h"

// Decoded basic-block cache for ROM-resident Thumb code.
//
// Each cache entry is a sequence of pre-decoded micro-ops -- a handler
// pointer plus the raw opcode -- ending at any branch/SWI/PC-write. The
// inner Thumb dispatch in arm_exec consults this cache before falling
// back to single-step interpretation. On a hit, the executor walks the
// micro-op array, skipping the per-instruction fetch (rom_buffer
// chunk lookup + native load) and the dispatch switch through
// thumb_proc[].
//
// Scope: ROM-resident Thumb only (regions 0x08-0x0D). RAM-resident code
// would need write-tracking to invalidate stale blocks; that's a later
// phase. ARM mode is not cached here either.
//
// Allocation: directory and uop pool come from extram via kmalloc at
// boot. If allocation fails, thumb_block_enabled stays false and the
// interpreter runs unchanged -- no crash, no perf gain.

// Per-instruction record. 12 bytes.
//
// The handler takes a pointer to its own thumb_uop_t. Specialised
// handlers (for the hottest opcodes) read pre-decoded operand fields
// (arg_a / arg_b / arg_c) directly, skipping the per-execution
// arm_op-read + bit-extraction work the legacy interpreter handlers
// do every time. Non-specialised opcodes use the t16_dec_call_legacy
// fallback handler, which sets the global arm_op from raw_op and
// dispatches into the existing thumb_proc[] table.
struct thumb_uop_s;
typedef void (*thumb_uop_handler_t)(const struct thumb_uop_s *uop);

typedef struct thumb_uop_s {
    thumb_uop_handler_t handler;
    uint16_t raw_op;     // for the legacy fallback path
    uint8_t  cycles;     // pre-computed waitstate-aware cycle cost
    uint8_t  pad;
    // Specialised-handler operand fields. Layout depends on the
    // specific handler; common conventions:
    //   ALU imm8 (mov/cmp/add/sub Rd, #imm8): arg_a=Rd, arg_b=imm8.
    //   Shift imm5 (lsl/lsr/asr Rd, Rm, #imm5): arg_a=Rd, arg_b=Rm,
    //     arg_c=imm5 (with the immediate=0 -> 32 mapping baked in for
    //     LSR/ASR).
    //   B imm11: arg_c holds the sign-extended (imm11<<1) byte offset.
    uint8_t  arg_a;
    uint8_t  arg_b;
    uint16_t arg_c;
} thumb_uop_t;

// Cache directory entry. 16 bytes.
//
// page_idx + page_gen are used for RAM-resident blocks (IWRAM 0x03,
// EWRAM 0x02) so write-side invalidation can detect when cached code
// has been overwritten. page_idx == 0xFFFF means "ROM block, no page
// check needed" -- ROM is read-only so cached blocks stay valid until
// the pool wraps (the existing `generation` field handles that).
typedef struct {
    uint32_t start_pc;       // sentinel 0xFFFFFFFF = empty
    uint16_t length;
    uint16_t total_cycles;
    uint16_t ops_offset;     // index into uop pool
    uint16_t generation;     // matches pool's current_gen, else stale
    uint16_t page_idx;       // RAM page; 0xFFFF for ROM blocks
    uint16_t page_gen;       // RAM page gen at decode time (ignored for ROM)
    // Phase 0 JIT slot. NULL = walk uops via thumb_uop_pool[ops_offset];
    // non-NULL = call this SH4 routine with &arm_r in R4. The executor
    // pre-credits total_cycles before the call. See docs/SH4_JIT_PLAN.md.
    native_block_fn_t native_entry;
} thumb_block_t;

// Capacity. Sized so the whole structure fits in ~128 KB extram.
#define THUMB_BLOCK_DIR_BITS  12
#define THUMB_BLOCK_DIR_SIZE  (1u << THUMB_BLOCK_DIR_BITS)
#define THUMB_BLOCK_DIR_MASK  (THUMB_BLOCK_DIR_SIZE - 1u)

#define THUMB_UOP_POOL_SIZE   8192u   // micro-op slots
#define THUMB_BLOCK_MAX_LEN   64u     // hard cap per block

// Returns true iff initialisation succeeded and block caching is live.
extern bool thumb_block_enabled;

// Pool storage exposed for the inline executor in arm.c (single-source
// definitions live in thumb_block.c).
extern thumb_block_t *thumb_block_dir;
extern thumb_uop_t   *thumb_uop_pool;
extern uint16_t       thumb_block_current_gen;

// Initialise; allocate from extram. Safe to call once after arm_init().
// Returns false if allocation failed (thumb_block_enabled stays false).
bool thumb_block_init(void);
void thumb_block_uninit(void);

// Look up a block for the given Thumb-instruction PC. Returns NULL if
// the PC is outside ROM, the directory slot is empty/stale, or it
// belongs to a different PC (hash collision).
const thumb_block_t *thumb_block_lookup(uint32_t inst_pc);

// Decode a fresh block starting at inst_pc and install it in the
// directory (overwriting any colliding entry). Returns the new block,
// or NULL if decoding failed (e.g. inst_pc not in ROM, undefined op
// at start, pool exhausted and even after a wrap+gen-bump there's no
// room -- shouldn't happen for sane block lengths).
const thumb_block_t *thumb_block_decode(uint32_t inst_pc);

// Direct-mapped index from Thumb-instruction PC.
static inline uint32_t thumb_block_hash(uint32_t inst_pc) {
    return (inst_pc >> 1) & THUMB_BLOCK_DIR_MASK;
}

// Bump the page generation for the page containing `addr`. Called from
// the IWRAM/EWRAM write paths in arm_mem.c so that cached RAM blocks
// covering this page are detected as stale on next lookup.
//
// Cheap: one array index + increment. Safe to call when the cache is
// disabled (the page-gen array lives in BSS, always allocated). For
// non-RAM addresses this is a no-op (page index resolves to 0xFFFF).
void thumb_block_invalidate_page(uint32_t addr);

// Walk the directory and clear native_entry on every slot. Called from
// jit_arena_recycle() when the JIT arena fills and we want to reuse it
// for fresh codegen -- old native_entry pointers would refer to about-
// to-be-overwritten arena bytes, so they must be NULL'd before the
// cursor is reset. Safe to call mid-decode; the executor's
// native_entry check just sees NULL and takes the interpreter path.
void thumb_block_clear_native_entries(void);

#endif // THUMB_BLOCK_H
