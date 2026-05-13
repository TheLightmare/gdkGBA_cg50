#ifndef CODE_BLOCK_H
#define CODE_BLOCK_H

#include <stdint.h>

#include "arm_mem.h"   // ewram_mask -- referenced from inline code_page_idx

// Shared page-generation tracking for the Thumb and ARM block caches.
//
// A "page" is 256 bytes. Pages 0..127 cover IWRAM (32 KB), pages
// 128..1151 cover EWRAM (256 KB max). On every RAM write, arm_mem.c's
// write paths call code_block_invalidate_page(addr), which bumps the
// gen for the page containing addr. Cached blocks store the page gen
// they observed at decode time and compare against the current gen on
// lookup; a mismatch means the block's source bytes have been
// overwritten since decode, so the cached entry is stale.
//
// Both the Thumb and ARM caches consult the same array because writes
// invalidate code regardless of which mode that code is in. Splitting
// the gen counters per-mode would let a Thumb write silently bypass
// the ARM cache's freshness check (or vice versa), so the array is
// single-source.

#define CODE_PAGE_BITS    8u
#define CODE_PAGE_SIZE    (1u << CODE_PAGE_BITS)
#define CODE_PAGE_MASK    (CODE_PAGE_SIZE - 1u)
#define CODE_IWRAM_PAGES  (0x8000u  / CODE_PAGE_SIZE)   // 128
#define CODE_EWRAM_PAGES  (0x40000u / CODE_PAGE_SIZE)   // 1024
#define CODE_TOTAL_PAGES  (CODE_IWRAM_PAGES + CODE_EWRAM_PAGES)
#define CODE_PAGE_NONE    0xFFFFu

// uint16_t: a page can be written 65535 times before the gen counter
// wraps. After wrap, a stale block whose stored gen happens to match
// the current gen would falsely pass the freshness check. Not a
// correctness bug in practice -- typical RAM pages don't see that
// many writes between block evictions -- but worth knowing.
extern uint16_t code_page_gen[CODE_TOTAL_PAGES];

// Map a guest address to its page index, or CODE_PAGE_NONE for
// non-RAM regions (ROM, BIOS, I/O, VRAM, etc.).
//
// Inline because invalidate_page is on the hot RAM-write path; the
// non-RAM early-out is a single byte compare + branch.
static inline uint16_t code_page_idx(uint32_t addr) {
    uint8_t reg = (addr >> 24) & 0xFu;
    if (reg == 0x03) {
        return (uint16_t)((addr & 0x7FFFu) >> CODE_PAGE_BITS);
    }
    if (reg == 0x02) {
        return (uint16_t)(CODE_IWRAM_PAGES +
                          ((addr & ewram_mask) >> CODE_PAGE_BITS));
    }
    return CODE_PAGE_NONE;
}

// Bump the page generation for the page containing `addr`. Called
// from the IWRAM/EWRAM write paths in arm_mem.c. Cheap: one inline
// index calculation plus one array increment. Safe to call for
// non-RAM addresses (resolves to CODE_PAGE_NONE and short-circuits).
void code_block_invalidate_page(uint32_t addr);

#endif // CODE_BLOCK_H
