#ifndef ARM_BLOCK_H
#define ARM_BLOCK_H

#include <stdint.h>
#include <stdbool.h>

// Decoded basic-block cache for ARM-mode code. Mirrors thumb_block.h
// but with 4-byte instructions and a per-uop condition-code field, so
// the executor can run conditional ARM instructions inside a block
// without splitting at every non-AL opcode.
//
// Bench profiling showed ARM mode taking 40-75% of arm_exec time in
// heavy scenes (snap 6: 50751 us / 67686 us = 75%), with no block
// cache equivalent to Thumb's. This file adds one.
//
// Scope: ROM (0x08-0x0D) + RAM (IWRAM 0x03, EWRAM 0x02). Pages are
// invalidated via the shared thumb_block_invalidate_page() hook --
// the same page-gen array tracks writes for both caches.

struct arm_uop_s;
typedef void (*arm_uop_handler_t)(const struct arm_uop_s *uop);

// 16 bytes per uop. raw_op is the full 32-bit instruction for the
// legacy fallback; cond is the 4-bit ARM condition (0..15). The
// executor short-circuits cond==14 (AL) before any arm_cond call.
typedef struct arm_uop_s {
    arm_uop_handler_t handler;
    uint32_t raw_op;
    uint8_t  cycles;
    uint8_t  cond;
    uint8_t  arg_a;
    uint8_t  arg_b;
    uint32_t arg_c;   // wide enough for imm12 / imm24 / address parts
} arm_uop_t;

// Cache directory entry, 16 bytes. Same layout as thumb_block_t so
// the executor pattern carries over cleanly.
typedef struct {
    uint32_t start_pc;       // sentinel 0xFFFFFFFF = empty
    uint16_t length;
    uint16_t total_cycles;
    uint16_t ops_offset;
    uint16_t generation;
    uint16_t page_idx;       // 0xFFFF for ROM blocks
    uint16_t page_gen;
} arm_block_t;

// Capacity. ARM directory same size as Thumb (4096 slots × 16 B = 64 KB).
// Uop pool is larger because ARM blocks are 4-byte instructions -- same
// instruction-count budget needs 2x the byte count, but uop_t is
// already larger (16 B vs 12 B). Keep counts close so total extram
// footprint stays manageable.
#define ARM_BLOCK_DIR_BITS  12
#define ARM_BLOCK_DIR_SIZE  (1u << ARM_BLOCK_DIR_BITS)
#define ARM_BLOCK_DIR_MASK  (ARM_BLOCK_DIR_SIZE - 1u)

#define ARM_UOP_POOL_SIZE   8192u   // micro-op slots
#define ARM_BLOCK_MAX_LEN   64u     // hard cap per block

extern bool arm_block_enabled;
extern arm_block_t *arm_block_dir;
extern arm_uop_t   *arm_uop_pool;
extern uint16_t     arm_block_current_gen;

bool arm_block_init(void);
void arm_block_uninit(void);

const arm_block_t *arm_block_lookup(uint32_t inst_pc);
const arm_block_t *arm_block_decode(uint32_t inst_pc);

// Direct-mapped index from ARM-instruction PC. ARM instructions are
// 4-byte aligned, so shift by 2 to spread the entropy.
static inline uint32_t arm_block_hash(uint32_t inst_pc) {
    return (inst_pc >> 2) & ARM_BLOCK_DIR_MASK;
}

#endif // ARM_BLOCK_H
