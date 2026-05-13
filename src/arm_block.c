#include "arm_block.h"

#include <stdlib.h>
#include <string.h>

#include <gint/kmalloc.h>

#include "code_block.h"
#include "extram.h"

// Skeleton phase: structures + allocation only. Lookup returns NULL
// (so the arm_exec dispatcher's NULL short-circuit falls through to
// the legacy single-step path). The boundary detector + decoder
// lands in the next step on this branch; the executor + dispatcher
// integration after that.

bool         arm_block_enabled     = false;
arm_block_t *arm_block_dir         = NULL;
arm_uop_t   *arm_uop_pool          = NULL;
uint16_t     arm_block_current_gen = 1;

bool arm_block_init(void) {
    size_t dir_bytes  = ARM_BLOCK_DIR_SIZE * sizeof(arm_block_t);
    size_t pool_bytes = ARM_UOP_POOL_SIZE  * sizeof(arm_uop_t);

    if (extram_available) {
        arm_block_dir = (arm_block_t *)kmalloc(dir_bytes,  "extram");
        arm_uop_pool  = (arm_uop_t   *)kmalloc(pool_bytes, "extram");
    } else {
        arm_block_dir = (arm_block_t *)malloc(dir_bytes);
        arm_uop_pool  = (arm_uop_t   *)malloc(pool_bytes);
    }
    if (!arm_block_dir || !arm_uop_pool) {
        arm_block_uninit();
        return false;
    }

    // Empty all directory slots. start_pc = 0xFFFFFFFF means "vacant";
    // generation = 0 distinguishes from any decoded block (gen starts
    // at 1, matching the Thumb cache convention).
    for (uint32_t i = 0; i < ARM_BLOCK_DIR_SIZE; i++) {
        arm_block_dir[i].start_pc     = 0xFFFFFFFFu;
        arm_block_dir[i].length       = 0;
        arm_block_dir[i].total_cycles = 0;
        arm_block_dir[i].ops_offset   = 0;
        arm_block_dir[i].generation   = 0;
        arm_block_dir[i].page_idx     = CODE_PAGE_NONE;
        arm_block_dir[i].page_gen     = 0;
    }

    arm_block_current_gen = 1;
    // arm_block_enabled stays false until the executor lands. The
    // allocation succeeded so a later flip is safe; for the skeleton
    // phase the dispatcher must not consult the cache, since lookup
    // would return NULL anyway but the cost of the lookup itself is
    // pointless overhead.
    return true;
}

void arm_block_uninit(void) {
    arm_block_enabled = false;
    if (arm_block_dir) { kfree(arm_block_dir); arm_block_dir = NULL; }
    if (arm_uop_pool)  { kfree(arm_uop_pool);  arm_uop_pool  = NULL; }
}

const arm_block_t *arm_block_lookup(uint32_t inst_pc) {
    // Skeleton: always returns NULL. Real implementation in step 3.
    (void)inst_pc;
    return NULL;
}

const arm_block_t *arm_block_decode(uint32_t inst_pc) {
    // Skeleton: stub. Boundary detector + decoder lands in step 3.
    (void)inst_pc;
    return NULL;
}
