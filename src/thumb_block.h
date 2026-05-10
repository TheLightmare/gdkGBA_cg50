#ifndef THUMB_BLOCK_H
#define THUMB_BLOCK_H

#include <stdint.h>
#include <stdbool.h>

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

// Per-instruction record. 8 bytes.
typedef struct {
    void (*handler)();   // entry from thumb_proc[op >> 5]
    uint16_t raw_op;     // restored to global arm_op before calling handler
    uint8_t  cycles;     // pre-computed waitstate-aware cycle cost
    uint8_t  pad;
} thumb_uop_t;

// Cache directory entry. 16 bytes.
typedef struct {
    uint32_t start_pc;       // sentinel 0xFFFFFFFF = empty
    uint16_t length;
    uint16_t total_cycles;
    uint16_t ops_offset;     // index into uop pool
    uint16_t generation;     // matches pool's current_gen, else stale
    uint16_t _pad;
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

#endif // THUMB_BLOCK_H
