#include "thumb_block.h"

#include <stdlib.h>
#include <string.h>

#include <gint/kmalloc.h>

#include "arm_mem.h"      // rom_buffer, wram_chip, wram_board, ewram_mask
#include "build_flags.h"
#include "extram.h"
#include "io.h"           // ws_s_t16
#if GBA_JIT_THUMB
#  include "thumb_jit.h"
#endif
#include "mem_swizzle.h"
#include "rom_buffer.h"   // rom_buffer_read_16_fast

bool thumb_block_enabled = false;
thumb_block_t *thumb_block_dir   = NULL;
thumb_uop_t   *thumb_uop_pool    = NULL;
uint16_t       thumb_block_current_gen = 1;
static uint32_t thumb_uop_pool_head = 0;

// Page-generation tracking for RAM-resident blocks.
//
// A "page" is 64 bytes (Phase 1 chunk 7 -- previously 256). The smaller
// granularity dramatically reduces false-positive invalidations: a data
// write to a page-co-tenant variable no longer marks unrelated code in
// the same page as stale. Code copied into IWRAM and read-only after
// (a common GBA cart pattern) now stays JIT'd across the cart's data
// writes to neighbouring memory.
//
// Pages 0..511 cover IWRAM (32 KB), pages 512..4607 cover EWRAM
// (256 KB max). The arm_mem.c write paths bump the gen for the page
// they wrote into; cached RAM blocks store the page gen they saw at
// decode time, and the lookup compares it against the current gen to
// detect overwrites.
//
// uint16_t means a page can be written 65535 times before the gen
// counter wraps. After wrap, a stale block whose stored gen happens
// to match the current gen would falsely pass the freshness check
// (cart misbehaves until the block is naturally evicted by pool
// rotation -- not a correctness issue per se but worth knowing).
#define THUMB_PAGE_BITS    6u
#define THUMB_PAGE_SIZE    (1u << THUMB_PAGE_BITS)
#define THUMB_PAGE_MASK    (THUMB_PAGE_SIZE - 1u)
#define THUMB_IWRAM_PAGES  (0x8000u / THUMB_PAGE_SIZE)        // 512
#define THUMB_EWRAM_PAGES  (0x40000u / THUMB_PAGE_SIZE)       // 4096
#define THUMB_TOTAL_PAGES  (THUMB_IWRAM_PAGES + THUMB_EWRAM_PAGES)
#define THUMB_PAGE_NONE    0xFFFFu

// Non-static: arm_block.c reads this array directly for its own lookup
// freshness check. Both caches share the same page identity so a single
// invalidation hook in arm_mem.c covers both.
uint16_t thumb_page_gen[THUMB_TOTAL_PAGES];

// Map a guest address to its page index in thumb_page_gen, or
// THUMB_PAGE_NONE for non-RAM regions.
static inline uint16_t ram_page_idx(uint32_t addr) {
    uint8_t reg = (addr >> 24) & 0xFu;
    if (reg == 0x03) {
        return (uint16_t)((addr & 0x7FFFu) >> THUMB_PAGE_BITS);
    }
    if (reg == 0x02) {
        return (uint16_t)(THUMB_IWRAM_PAGES +
                          ((addr & ewram_mask) >> THUMB_PAGE_BITS));
    }
    return THUMB_PAGE_NONE;
}

void thumb_block_invalidate_page(uint32_t addr) {
    uint16_t idx = ram_page_idx(addr);
    if (idx != THUMB_PAGE_NONE) {
        thumb_page_gen[idx]++;
    }
}

// thumb_proc[] is defined in arm.c. The Phase 4b legacy fallback
// handler dispatches through it for opcodes without a specialised
// uop-aware handler.
extern void (*thumb_proc[2048])();

// Phase 4b specialised handlers (defined in arm.c near the legacy
// handlers so they have access to the static inline helper functions).
extern void t16_dec_mov_imm8 (const thumb_uop_t *uop);
extern void t16_dec_cmp_imm8 (const thumb_uop_t *uop);
extern void t16_dec_add_imm8 (const thumb_uop_t *uop);
extern void t16_dec_sub_imm8 (const thumb_uop_t *uop);
extern void t16_dec_lsl_imm5 (const thumb_uop_t *uop);
extern void t16_dec_lsr_imm5 (const thumb_uop_t *uop);
extern void t16_dec_asr_imm5 (const thumb_uop_t *uop);
extern void t16_dec_ldr_imm5 (const thumb_uop_t *uop);
extern void t16_dec_str_imm5 (const thumb_uop_t *uop);
extern void t16_dec_ldr_pc8  (const thumb_uop_t *uop);
extern void t16_dec_add_imm3 (const thumb_uop_t *uop);
extern void t16_dec_sub_imm3 (const thumb_uop_t *uop);
extern void t16_dec_add_reg  (const thumb_uop_t *uop);
extern void t16_dec_sub_reg  (const thumb_uop_t *uop);
extern void t16_dec_b_imm11  (const thumb_uop_t *uop);
extern void t16_dec_b_cond   (const thumb_uop_t *uop);
extern void t16_dec_call_legacy(const thumb_uop_t *uop);

// Pick the right uop handler for `op` and fill in the operand fields it
// expects. Falls through to t16_dec_call_legacy for opcodes without a
// specialised handler -- behaviour identical to the legacy interpreter.
static void decode_thumb_op(uint16_t op, thumb_uop_t *out) {
    out->raw_op = op;
    out->arg_a  = 0;
    out->arg_b  = 0;
    out->arg_c  = 0;

    uint8_t top5 = op >> 11;
    uint16_t imm5;

    switch (top5) {
        case 0b00000:  // LSL Rd, Rm, #imm5  (also covers MOV Rd, Rm
                       // when imm5==0 -- arm_lsl with shift=0 produces
                       // the same Rd-write + N/Z-update + C-unchanged
                       // effect as t16_mov_rd3).
            out->handler = t16_dec_lsl_imm5;
            out->arg_a   = op & 0x7;
            out->arg_b   = (op >> 3) & 0x7;
            out->arg_c   = (op >> 6) & 0x1f;
            return;
        case 0b00001:  // LSR Rd, Rm, #imm5  (imm5==0 means shift by 32)
            out->handler = t16_dec_lsr_imm5;
            out->arg_a   = op & 0x7;
            out->arg_b   = (op >> 3) & 0x7;
            imm5 = (op >> 6) & 0x1f;
            out->arg_c   = (imm5 == 0) ? 32 : imm5;
            return;
        case 0b00010:  // ASR Rd, Rm, #imm5  (imm5==0 means shift by 32)
            out->handler = t16_dec_asr_imm5;
            out->arg_a   = op & 0x7;
            out->arg_b   = (op >> 3) & 0x7;
            imm5 = (op >> 6) & 0x1f;
            out->arg_c   = (imm5 == 0) ? 32 : imm5;
            return;
        case 0b00011: {
            // ADD/SUB reg or imm3:
            //   bit 10 = I (1 = immediate, 0 = register)
            //   bit 9  = Op (0 = ADD, 1 = SUB)
            //   bits[8:6] = imm3 or Rm
            //   bits[5:3] = Rn
            //   bits[2:0] = Rd
            bool is_imm = (op >> 10) & 1;
            bool is_sub = (op >>  9) & 1;
            out->arg_a = op & 0x7;
            out->arg_b = (op >> 3) & 0x7;
            out->arg_c = (op >> 6) & 0x7;
            if (is_imm) {
                out->handler = is_sub ? t16_dec_sub_imm3 : t16_dec_add_imm3;
            } else {
                out->handler = is_sub ? t16_dec_sub_reg  : t16_dec_add_reg;
            }
            return;
        }
        case 0b00100:  // MOV Rd, #imm8
            out->handler = t16_dec_mov_imm8;
            out->arg_a   = (op >> 8) & 0x7;
            out->arg_b   = op & 0xff;
            return;
        case 0b00101:  // CMP Rd, #imm8
            out->handler = t16_dec_cmp_imm8;
            out->arg_a   = (op >> 8) & 0x7;
            out->arg_b   = op & 0xff;
            return;
        case 0b00110:  // ADD Rd, #imm8
            out->handler = t16_dec_add_imm8;
            out->arg_a   = (op >> 8) & 0x7;
            out->arg_b   = op & 0xff;
            return;
        case 0b00111:  // SUB Rd, #imm8
            out->handler = t16_dec_sub_imm8;
            out->arg_a   = (op >> 8) & 0x7;
            out->arg_b   = op & 0xff;
            return;
        case 0b01001:  // LDR Rd, [PC, #imm8*4]
            out->handler = t16_dec_ldr_pc8;
            out->arg_a   = (op >> 8) & 0x7;
            out->arg_c   = (uint16_t)((op & 0xff) << 2);
            return;
        case 0b01100:  // STR Rd, [Rn, #imm5*4]
            out->handler = t16_dec_str_imm5;
            out->arg_a   = op & 0x7;
            out->arg_b   = (op >> 3) & 0x7;
            out->arg_c   = (uint16_t)(((op >> 6) & 0x1f) << 2);
            return;
        case 0b01101:  // LDR Rd, [Rn, #imm5*4]
            out->handler = t16_dec_ldr_imm5;
            out->arg_a   = op & 0x7;
            out->arg_b   = (op >> 3) & 0x7;
            out->arg_c   = (uint16_t)(((op >> 6) & 0x1f) << 2);
            return;
        case 0b11010:
        case 0b11011: {
            // B<cond> #imm8 -- conditional branch. Bits 11..8 are the
            // 4-bit ARM condition; cond=14 (0xDE) is UDF and cond=15
            // (0xDF) is SWI -- both encoded in this top5 range, but
            // routed to legacy.
            //
            // Sign-extend imm8 (bits 7..0) and shift left by 1 to get
            // the byte offset in [-256, +254]. Fits in int16_t.
            uint8_t cond = (op >> 8) & 0xf;
            if (cond >= 14) {
                out->handler = t16_dec_call_legacy;
                return;
            }
            int32_t imm = ((int32_t)((uint32_t)op << 24)) >> 23;
            out->handler = t16_dec_b_cond;
            out->arg_a   = cond;
            out->arg_c   = (uint16_t)imm;
            return;
        }
        case 0b11100: {
            // B #imm11 -- compute the sign-extended (imm11 << 1) byte
            // offset at decode time. Range is [-2048, +2046], fits in
            // int16_t. Stored as uint16_t; the handler casts back to
            // int16_t to recover the sign.
            int32_t imm = ((int32_t)((uint32_t)op << 21)) >> 20;
            out->handler = t16_dec_b_imm11;
            out->arg_c   = (uint16_t)imm;
            return;
        }
        default:
            out->handler = t16_dec_call_legacy;
            return;
    }
}

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
        thumb_block_dir[i].page_idx     = THUMB_PAGE_NONE;
        thumb_block_dir[i].page_gen     = 0;
        thumb_block_dir[i].native_entry = NULL;
    }

    // Reset all page generations. Static BSS already zero, but be
    // explicit -- we'll bump these on every RAM write.
    for (uint32_t i = 0; i < THUMB_TOTAL_PAGES; i++) {
        thumb_page_gen[i] = 0;
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

__attribute__((noinline))
const thumb_block_t *thumb_block_lookup(uint32_t inst_pc) {
    if (!thumb_block_enabled) return NULL;
    uint8_t reg = (inst_pc >> 24) & 0xFu;
    bool is_rom = (reg >= 0x8 && reg <= 0xD);
    bool is_ram = (reg == 0x02 || reg == 0x03);
    if (!is_rom && !is_ram) return NULL;

    thumb_block_t *b = &thumb_block_dir[thumb_block_hash(inst_pc)];
    if (b->start_pc != inst_pc) return NULL;
    if (b->generation != thumb_block_current_gen) return NULL;

    // For RAM blocks, also confirm the page hasn't been written to since
    // we decoded. ROM blocks have page_idx == THUMB_PAGE_NONE and skip
    // this check.
    if (b->page_idx != THUMB_PAGE_NONE) {
        if (thumb_page_gen[b->page_idx] != b->page_gen) return NULL;
    }

    return b;
}

// Read a Thumb halfword for the decoder. ROM goes through the Phase 1
// chunk cache; IWRAM/EWRAM go through the swizzled fast path.
static inline uint16_t decode_read_op(uint32_t pc) {
    uint8_t reg = (pc >> 24) & 0xFu;
    if (reg >= 0x8 && reg <= 0xD) {
        return rom_buffer_read_16_fast(&rom_buffer, pc);
    }
    if (reg == 0x03) {
        return mem_swz_read_h(wram_chip, pc & 0x7FFFu);
    }
    if (reg == 0x02) {
        return mem_swz_read_h(wram_board, pc & ewram_mask);
    }
    return 0;
}

__attribute__((noinline))
const thumb_block_t *thumb_block_decode(uint32_t inst_pc) {
    if (!thumb_block_enabled) return NULL;
    uint8_t reg = (inst_pc >> 24) & 0xFu;
    bool is_rom = (reg >= 0x8 && reg <= 0xD);
    bool is_ram = (reg == 0x02 || reg == 0x03);
    if (!is_rom && !is_ram) return NULL;

    // For RAM blocks, snapshot the page identity + generation now so
    // the lookup-side freshness check has something to compare. The
    // block decoder also caps the block at the next 256-byte page
    // boundary so we never need to track per-page state for >1 page.
    uint16_t page_idx = THUMB_PAGE_NONE;
    uint16_t page_gen = 0;
    uint32_t page_end_pc = 0xFFFFFFFFu;
    if (is_ram) {
        page_idx = ram_page_idx(inst_pc);
        if (page_idx == THUMB_PAGE_NONE) return NULL;  // shouldn't hit
        page_gen = thumb_page_gen[page_idx];
        page_end_pc = (inst_pc | THUMB_PAGE_MASK) + 1u;
    }

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
    // Cycle cost per instruction in this region. For ROM regions, this
    // is the cart's WAITCNT-derived value; RAM is fast (1 for IWRAM,
    // 3 for EWRAM as approximated by ws_s_t16's index 0 fallback).
    uint8_t  region_ws;
    if (is_rom) region_ws = ws_s_t16[(pc >> 25) & 3];
    else if (reg == 0x03) region_ws = 1;     // IWRAM: 1-cycle access
    else                  region_ws = 3;     // EWRAM: bus is 16-bit, slow

    while (length < THUMB_BLOCK_MAX_LEN && pc < page_end_pc) {
        uint8_t r = (pc >> 24) & 0xFu;
        bool r_rom = (r >= 0x8 && r <= 0xD);
        bool r_ram = (r == 0x02 || r == 0x03);
        // Bail if we'd cross out of the start region (e.g., we somehow
        // wrap past the IWRAM end).
        if (is_rom && !r_rom) break;
        if (is_ram && !r_ram) break;

        uint16_t op = decode_read_op(pc);

        decode_thumb_op(op, &ops[length]);
        ops[length].cycles  = region_ws;
        ops[length].pad     = 0;

        total_cycles += region_ws;
        length++;

        if (t16_block_ends(op)) break;

        pc += 2;
        // Re-read region waitstate when the address crosses into a
        // different waitstate domain (rare in practice for ROM; never
        // for RAM since RAM blocks are page-bounded).
        if (is_rom) region_ws = ws_s_t16[(pc >> 25) & 3];
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
    slot->page_idx     = page_idx;
    slot->page_gen     = page_gen;
    // Decoder default: NULL native_entry so the executor takes the
    // interpreter path. Explicit because we may be overwriting an entry
    // from a different PC that hashed here.
    slot->native_entry = NULL;

    thumb_uop_pool_head += length;

#if GBA_JIT_THUMB
    // Phase 1 Chunk 1: try to JIT-compile this block. On any failure
    // (arena full, block too long for the literal-pool disp, etc.)
    // native_entry stays NULL and the executor falls back to walking
    // the uop list.
    (void)thumb_jit_compile_block(slot);
#endif

    return slot;
}
