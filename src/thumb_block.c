#include "thumb_block.h"

#include <stdlib.h>
#include <string.h>

#include <gint/kmalloc.h>

#include "arm_mem.h"      // rom_buffer global
#include "extram.h"
#include "io.h"           // ws_s_t16
#include "rom_buffer.h"   // rom_buffer_read_16_fast

bool thumb_block_enabled = false;
thumb_block_t *thumb_block_dir   = NULL;
thumb_uop_t   *thumb_uop_pool    = NULL;
uint16_t       thumb_block_current_gen = 1;
static uint32_t thumb_uop_pool_head = 0;

// thumb_proc[] is defined in arm.c. We dispatch through it for every
// in-block opcode; the inlined fast paths in t16_step's switch don't
// help once we're going through a function pointer anyway. A later
// phase can swap in specialised handlers for hot opcodes.
extern void (*thumb_proc[2048])();

// Returns true if the opcode could change PC (or otherwise needs a
// fresh pipeline at exit). Conservative -- false negatives would let
// the block run past a branch with stale state, false positives just
// shorten the block.
//
// Matches the Thumb v4T encoding map:
//   0x44..0x47  high-reg ALU (ADD/CMP/MOV/BX-BLX) -- could write R15
//   0xBD        POP with R15 in mask
//   0xBE        BKPT
//   0xBF        unallocated / hint instructions (be safe)
//   0xD0..0xDF  B<cond> + SWI
//   0xE0..0xE7  B unconditional
//   0xE8..0xEF  BLX suffix
//   0xF8..0xFF  BL suffix
//
// 0xF0..0xF7 (BL/BLX prefix) is intentionally NOT block-ending: the
// suffix that follows is, so the block naturally ends there.
static bool t16_block_ends(uint16_t op) {
    uint8_t hi = (uint8_t)(op >> 8);
    if (hi >= 0x44 && hi <= 0x47) return true;
    if (hi == 0xBD || hi == 0xBE || hi == 0xBF) return true;
    if (hi >= 0xD0 && hi <= 0xEF) return true;
    if (hi >= 0xF8) return true;
    return false;
}

bool thumb_block_init(void) {
    size_t dir_bytes  = THUMB_BLOCK_DIR_SIZE * sizeof(thumb_block_t);
    size_t pool_bytes = THUMB_UOP_POOL_SIZE * sizeof(thumb_uop_t);

    if (extram_available) {
        thumb_block_dir = (thumb_block_t *)kmalloc(dir_bytes,  "extram");
        thumb_uop_pool  = (thumb_uop_t   *)kmalloc(pool_bytes, "extram");
    } else {
        thumb_block_dir = (thumb_block_t *)malloc(dir_bytes);
        thumb_uop_pool  = (thumb_uop_t   *)malloc(pool_bytes);
    }
    if (!thumb_block_dir || !thumb_uop_pool) {
        thumb_block_uninit();
        return false;
    }

    // Empty all directory slots. start_pc = 0xFFFFFFFF means "vacant";
    // generation = 0 distinguishes from any decoded block (gen starts at 1).
    for (uint32_t i = 0; i < THUMB_BLOCK_DIR_SIZE; i++) {
        thumb_block_dir[i].start_pc     = 0xFFFFFFFFu;
        thumb_block_dir[i].length       = 0;
        thumb_block_dir[i].total_cycles = 0;
        thumb_block_dir[i].ops_offset   = 0;
        thumb_block_dir[i].generation   = 0;
    }

    thumb_uop_pool_head      = 0;
    thumb_block_current_gen  = 1;
    thumb_block_enabled      = true;
    return true;
}

void thumb_block_uninit(void) {
    thumb_block_enabled = false;
    if (thumb_block_dir) { kfree(thumb_block_dir); thumb_block_dir = NULL; }
    if (thumb_uop_pool)  { kfree(thumb_uop_pool);  thumb_uop_pool  = NULL; }
}

const thumb_block_t *thumb_block_lookup(uint32_t inst_pc) {
    if (!thumb_block_enabled) return NULL;
    uint8_t reg = (inst_pc >> 24) & 0xFu;
    if (reg < 0x8 || reg > 0xD) return NULL;

    thumb_block_t *b = &thumb_block_dir[thumb_block_hash(inst_pc)];
    if (b->start_pc == inst_pc &&
        b->generation == thumb_block_current_gen) {
        return b;
    }
    return NULL;
}

const thumb_block_t *thumb_block_decode(uint32_t inst_pc) {
    if (!thumb_block_enabled) return NULL;
    uint8_t reg = (inst_pc >> 24) & 0xFu;
    if (reg < 0x8 || reg > 0xD) return NULL;

    // Reserve pool space. If we'd exceed the pool, wrap the head and bump
    // the generation -- every directory entry referencing the soon-to-be-
    // overwritten range becomes stale via gen mismatch.
    if (thumb_uop_pool_head + THUMB_BLOCK_MAX_LEN > THUMB_UOP_POOL_SIZE) {
        thumb_uop_pool_head = 0;
        thumb_block_current_gen++;
        // Wraparound: if generation hits 0 again it would alias old empty
        // slots. Bump past 0 so empty entries (gen=0) remain distinct.
        if (thumb_block_current_gen == 0) thumb_block_current_gen = 1;
    }

    thumb_uop_t *ops = thumb_uop_pool + thumb_uop_pool_head;
    uint32_t pc = inst_pc;
    uint16_t length = 0;
    uint16_t total_cycles = 0;
    uint8_t  region_ws = ws_s_t16[(pc >> 25) & 3];

    while (length < THUMB_BLOCK_MAX_LEN) {
        uint8_t r = (pc >> 24) & 0xFu;
        if (r < 0x8 || r > 0xD) break;

        // Read straight from the ROM cache. rom_buffer_read_16_fast hits
        // the last_chunk fast path on consecutive accesses; even on a
        // miss it goes through the Phase-1 hash table.
        uint16_t op = rom_buffer_read_16_fast(&rom_buffer, pc);

        ops[length].handler = thumb_proc[op >> 5];
        ops[length].raw_op  = op;
        ops[length].cycles  = region_ws;
        ops[length].pad     = 0;

        total_cycles += region_ws;
        length++;

        if (t16_block_ends(op)) break;

        pc += 2;
        // Re-read region waitstate when the address crosses into a
        // different waitstate domain (rare in practice -- ROM regions
        // 0x08-0x09 / 0x0A-0x0B / 0x0C-0x0D each have their own ws_s_t16
        // entry). Cheap: an XYRAM load.
        region_ws = ws_s_t16[(pc >> 25) & 3];
    }

    if (length == 0) return NULL;

    // Install in directory at the slot determined by inst_pc. May
    // overwrite an entry for a different PC that hashes here -- that
    // older block's pool storage is still valid until the pool wraps,
    // but it can't be looked up via the directory anymore.
    thumb_block_t *slot = &thumb_block_dir[thumb_block_hash(inst_pc)];
    slot->start_pc     = inst_pc;
    slot->length       = length;
    slot->total_cycles = total_cycles;
    slot->ops_offset   = (uint16_t)thumb_uop_pool_head;
    slot->generation   = thumb_block_current_gen;
    slot->_pad         = 0;

    thumb_uop_pool_head += length;
    return slot;
}
