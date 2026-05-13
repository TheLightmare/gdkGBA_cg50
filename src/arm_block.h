#ifndef ARM_BLOCK_H
#define ARM_BLOCK_H

#include <stdint.h>
#include <stdbool.h>

// Decoded basic-block cache for ARM-mode code.
//
// Mirrors the Thumb cache (thumb_block.{h,c}) but for 32-bit ARM ops.
// Diagnostic traces on Minish Cap showed ARM mode consuming 25-70% of
// inner-loop wall time depending on the scene (audio mixer + cart user
// IRQ handler at *(0x03007FFC) run in IWRAM-ARM). The Thumb cache
// already eliminates the per-instruction fetch + dispatch on the Thumb
// side; this one does the same for ARM.
//
// Skeleton-only at this stage: structures + init/uninit are live, but
// lookup always returns NULL and decode is a stub. The dispatcher in
// arm_exec is untouched until step 3 (decoder) and step 5 (executor).
//
// Coverage: ROM (regions 0x08-0x0D) and RAM (0x02 EWRAM, 0x03 IWRAM).
// BIOS (0x00) is skipped -- most BIOS calls are HLE'd, and the rest
// run too rarely to justify caching.
//
// Allocation: directory and uop pool come from extram via kmalloc at
// boot. If allocation fails, arm_block_enabled stays false and the
// single-step ARM dispatcher runs unchanged -- no crash, no perf gain.
// Sized smaller than the Thumb cache because the ARM hot working set
// is narrower (a handful of IWRAM routines, not whole-cart code).

// Per-instruction record. 8 bytes.
//
// Locked in earlier (see commit history / branch design notes):
//   handler : 4 bytes -- direct pointer; no per-instruction
//             arm_proc_idx_*[] indirection.
//   raw_op  : 4 bytes -- handlers set the global `arm_op` from this
//             before running, same shape as Thumb's legacy fallback.
//             Condition code (bits[31:28]) is extracted from raw_op
//             at executor time; storing it separately would just
//             bloat the uop.
//
// No `cycles` field: ARM-in-IWRAM is uniformly 1 cycle/op; ROM cost
// is per-region-stable and folded into arm_block_t.total_cycles at
// decode (one add per block-execution, same as Thumb).
struct arm_uop_s;
typedef void (*arm_uop_handler_t)(const struct arm_uop_s *uop);

typedef struct arm_uop_s {
    arm_uop_handler_t handler;
    uint32_t          raw_op;
} arm_uop_t;

// Cache directory entry. 16 bytes (same layout as thumb_block_t so
// future tooling that walks both can share code).
//
// page_idx + page_gen drive RAM-write invalidation via the shared
// code_page_gen[] array in code_block.h. ROM blocks set page_idx to
// CODE_PAGE_NONE and skip that check.
typedef struct {
    uint32_t start_pc;       // sentinel 0xFFFFFFFF = empty
    uint16_t length;
    uint16_t total_cycles;
    uint16_t ops_offset;     // index into arm_uop_pool
    uint16_t generation;     // matches pool's current_gen, else stale
    uint16_t page_idx;       // RAM page; CODE_PAGE_NONE for ROM blocks
    uint16_t page_gen;       // RAM page gen at decode time (ignored for ROM)
} arm_block_t;

// Capacity. Total extram footprint ~96 KB (dir 64 KB + pool 32 KB).
// Half the Thumb cache: the ARM hot working set on Minish Cap is
// roughly 12 KB of guest code (the 03004xxx-03006xxx range visible
// in steady-state PC traces); decoded-expanded at 8 bytes/uop this
// is ~24 KB of pool -- 32 KB gives modest headroom without locking
// up extram unnecessarily.
//
// Directory hash uses (inst_pc >> 2) since ARM instructions are
// 4-byte aligned. 12 bits -> 4096 slots; the visible PC range
// produces few collisions in practice.
#define ARM_BLOCK_DIR_BITS  12
#define ARM_BLOCK_DIR_SIZE  (1u << ARM_BLOCK_DIR_BITS)
#define ARM_BLOCK_DIR_MASK  (ARM_BLOCK_DIR_SIZE - 1u)

#define ARM_UOP_POOL_SIZE   4096u   // micro-op slots
#define ARM_BLOCK_MAX_LEN   64u     // hard cap per block

// True iff initialisation succeeded *and* the dispatcher is wired up
// (step 5). Skeleton phase: stays false even after init, so the
// dispatcher's NULL-lookup short-circuit keeps the single-step path
// running unchanged.
extern bool arm_block_enabled;

// Pool storage exposed for the future inline executor in arm.c.
extern arm_block_t *arm_block_dir;
extern arm_uop_t   *arm_uop_pool;
extern uint16_t     arm_block_current_gen;

// Allocate from extram. Safe to call once after arm_init(). Returns
// false on allocation failure (arm_block_enabled stays false).
bool arm_block_init(void);
void arm_block_uninit(void);

// Look up a block for the given ARM-instruction PC. Returns NULL if:
//   - the cache is not enabled
//   - PC is outside ROM/RAM coverage
//   - the directory slot is empty / stale (gen mismatch)
//   - it belongs to a different PC (hash collision)
//   - it's a RAM block whose page has been written since decode
//
// Skeleton: always returns NULL. Real implementation lands in step 3.
const arm_block_t *arm_block_lookup(uint32_t inst_pc);

// Decode a fresh block starting at inst_pc and install it. Returns
// the new block, or NULL on failure (e.g. inst_pc outside coverage,
// undefined first op, decoder skeleton). Skeleton: stub returns
// NULL.
const arm_block_t *arm_block_decode(uint32_t inst_pc);

// Direct-mapped hash from ARM-instruction PC. ARM ops are 4-byte
// aligned, so the bottom two bits are always zero; shift them out
// before masking.
static inline uint32_t arm_block_hash(uint32_t inst_pc) {
    return (inst_pc >> 2) & ARM_BLOCK_DIR_MASK;
}

#endif // ARM_BLOCK_H
