#include "arm_block.h"

#include <stdlib.h>
#include <string.h>

#include <gint/kmalloc.h>

#include "arm_mem.h"
#include "build_flags.h"
#include "extram.h"
#include "io.h"           // ws_s_arm
#if GBA_JIT_ARM
#  include "arm_jit.h"
#endif
#include "mem_swizzle.h"
#include "rom_buffer.h"
#include "thumb_block.h"  // shared page-gen for RAM-page invalidation

bool arm_block_enabled = false;
arm_block_t *arm_block_dir = NULL;
arm_uop_t   *arm_uop_pool  = NULL;
uint16_t     arm_block_current_gen = 1;
static uint32_t arm_uop_pool_head = 0;

// Same page tracking as the Thumb cache. The write hooks in arm_mem.c
// already call thumb_block_invalidate_page(addr) on every RAM write;
// that bumps the shared thumb_page_gen[] entry, and our lookup checks
// it the same way. No new write hooks needed.
#define ARM_PAGE_BITS    8u
#define ARM_PAGE_SIZE    (1u << ARM_PAGE_BITS)
#define ARM_PAGE_MASK    (ARM_PAGE_SIZE - 1u)
#define ARM_IWRAM_PAGES  (0x8000u / ARM_PAGE_SIZE)
#define ARM_EWRAM_PAGES  (0x40000u / ARM_PAGE_SIZE)
#define ARM_PAGE_NONE    0xFFFFu

// Mirror of the page-gen array. thumb_block.c owns the storage and
// the invalidation bump; we read it through this extern for the
// lookup-side freshness check.
extern uint16_t thumb_page_gen[];

static inline uint16_t ram_page_idx(uint32_t addr) {
    uint8_t reg = (addr >> 24) & 0xFu;
    if (reg == 0x03) {
        return (uint16_t)((addr & 0x7FFFu) >> ARM_PAGE_BITS);
    }
    if (reg == 0x02) {
        return (uint16_t)(ARM_IWRAM_PAGES +
                          ((addr & ewram_mask) >> ARM_PAGE_BITS));
    }
    return ARM_PAGE_NONE;
}

// Phase 1 fallback + Phase 2 specialised handlers live in arm.c so
// they can use the static inline arith/memio helpers there.
extern void arm_dec_call_legacy (const arm_uop_t *uop);
extern void arm_dec_b_imm24     (const arm_uop_t *uop);
extern void arm_dec_bl_imm24    (const arm_uop_t *uop);
extern void arm_dec_mov_imm     (const arm_uop_t *uop);
extern void arm_dec_add_imm     (const arm_uop_t *uop);
extern void arm_dec_sub_imm     (const arm_uop_t *uop);
extern void arm_dec_cmp_imm     (const arm_uop_t *uop);
extern void arm_dec_ldr_imm12   (const arm_uop_t *uop);
extern void arm_dec_str_imm12   (const arm_uop_t *uop);

// Decode one 32-bit ARM instruction into a uop. Returns true if
// specialised; falls back to arm_dec_call_legacy otherwise.
#define ARM_COND_UNCOND 0xF

static void decode_arm_op(uint32_t op, arm_uop_t *out) {
    out->raw_op = op;
    out->cond   = (uint8_t)(op >> 28);
    out->arg_a  = 0;
    out->arg_b  = 0;
    out->arg_c  = 0;
    out->handler = arm_dec_call_legacy;

    // cond==15 (UNCOND) encodings are an entirely different instruction
    // space (CDP2 / MCR2 / BLX-imm / PLD / etc.). Skip specialisation
    // and route them all through legacy.
    if (out->cond == ARM_COND_UNCOND) return;

    // Bits 27..25 form the major opcode group.
    uint32_t group = (op >> 25) & 0x7;

    switch (group) {
        case 0b101: {
            // Branch (B) and Branch with Link (BL). Bit 24 distinguishes:
            //   0 = B, 1 = BL. Both have a 24-bit signed offset that the
            // pipeline shift turns into a +8 displacement from PC.
            //
            // Both are block-ending (decode loop terminates after the
            // branch), but the uop still needs to be specialised so the
            // executor can update R15 and call arm_load_pipe without
            // going through arm_proc_handlers.
            int32_t imm = (int32_t)(op << 8) >> 6;  // sign-ext, *4
            out->arg_c = (uint32_t)imm;
            out->handler = (op & (1u << 24)) ? arm_dec_bl_imm24
                                             : arm_dec_b_imm24;
            return;
        }
        case 0b001: {
            // Data-processing immediate. Bits 24..21 are the opcode,
            // bit 20 is the S-bit. We specialise the 4 most common
            // (MOV/ADD/SUB/CMP) with S consistent with their canonical
            // use; other variants and S-mismatches fall through to
            // legacy.
            //
            //   arg_a = Rn (bits 19..16)
            //   arg_b = Rd (bits 15..12)
            //   arg_c = rotated imm32 (computed at decode time)
            uint8_t opc = (op >> 21) & 0xF;
            bool    s   = (op >> 20) & 1;
            uint8_t rd  = (op >> 12) & 0xF;
            uint8_t rn  = (op >> 16) & 0xF;

            // PC-writing data-proc instructions need to be block-ending
            // and aren't safe in the specialised handlers (they'd
            // need to invoke arm_load_pipe). Force legacy and end the
            // block.
            if (rd == 15) return;

            uint32_t imm8 = op & 0xFF;
            uint8_t  rot  = (op >> 8) & 0xF;
            uint32_t imm  = (imm8 >> (rot * 2)) | (imm8 << (32 - rot * 2));
            if (rot == 0) imm = imm8;

            out->arg_a = rn;
            out->arg_b = rd;
            out->arg_c = imm;

            switch (opc) {
                case 0b1101: if (!s) out->handler = arm_dec_mov_imm; break; // MOV (no S, avoids C-flag complications)
                case 0b0100: if ( s) out->handler = arm_dec_add_imm; break; // ADD,S
                case 0b0010: if ( s) out->handler = arm_dec_sub_imm; break; // SUB,S
                case 0b1010: if ( s) out->handler = arm_dec_cmp_imm; break; // CMP (always S)
            }
            return;
        }
        case 0b010: {
            // LDR / STR immediate offset.
            //   Bit 20 = L (1 = load, 0 = store)
            //   Bit 22 = B (1 = byte, 0 = word)  -- byte form falls back
            //   Bit 24 = P (1 = pre, 0 = post)
            //   Bit 23 = U (1 = add imm, 0 = sub imm)
            //   Bit 21 = W (writeback)
            //   imm12  = bits 11..0
            //
            // Specialise only the common shape: word access, pre-indexed,
            // no writeback. Everything else routes via legacy.
            bool L = (op >> 20) & 1;
            bool B = (op >> 22) & 1;
            bool P = (op >> 24) & 1;
            bool W = (op >> 21) & 1;
            uint8_t rn = (op >> 16) & 0xF;
            uint8_t rt = (op >> 12) & 0xF;

            // PC-relative loads are common (constant pools) but the
            // address depends on runtime R15, AND a load into R15
            // changes PC -- conservatively bail to legacy + end block.
            if (rn == 15 || rt == 15) return;
            if (B || !P || W) return;

            out->arg_a = rt;
            out->arg_b = rn;
            uint32_t imm12 = op & 0xFFF;
            // Encode sign bit of U into arg_c using two's-complement.
            int32_t off = (op & (1u << 23)) ? (int32_t)imm12 : -(int32_t)imm12;
            out->arg_c = (uint32_t)off;
            out->handler = L ? arm_dec_ldr_imm12 : arm_dec_str_imm12;
            return;
        }
        default:
            // All other groups: leave as legacy. Phase 2 can extend
            // (LDM/STM, shifted-register data-proc, MUL, etc.) as
            // profiling demands.
            return;
    }
}

// Returns true if `op` writes PC, branches, or otherwise needs the
// pipeline reset on exit. Conservative -- false negatives = stale
// state; false positives just shorten the block.
//
// We check (in priority):
//   * B/BL/BLX immediate          (bits 27..25 = 101)
//   * BX / BLX register form
//   * SWI                         (bits 27..24 = 1111)
//   * MSR to CPSR                 (changes mode/flags)
//   * Coprocessor (bits 27..24 = 1110, 1100, 1101) -- not on GBA but be safe
//   * Data-processing with Rd=15  (bits 27..25 = 000 or 001, Rd field)
//   * LDR with Rt=15
//   * LDM with bit 15 of register list set
//   * MUL family with Rd=15 (rare, but legal)
//   * Undefined  (bits 27..25 = 011 with bit 4 set -- this is the GBA undef space)
static bool arm_block_ends(uint32_t op) {
    uint32_t g = (op >> 25) & 0x7;       // major group bits 27..25
    uint32_t cb = (op >> 24) & 0xF;      // bits 27..24

    // Branches (B / BL): bits 27..25 = 101
    if (g == 0b101) return true;
    // SWI: bits 27..24 = 1111
    if (cb == 0b1111) return true;
    // Coprocessor / undef space (1100..1110): treat as block-ending.
    if (cb >= 0b1100 && cb <= 0b1110) return true;

    // BX / BLX register: bits 27..4 == 0x12FFF1 (BX) / 0x12FFF3 (BLX)
    if ((op & 0x0FFFFFD0u) == 0x012FFF10u) return true;

    // LDM/STM (block data transfer): bits 27..25 = 100. If R15 is in
    // the register list (bit 15 of low 16), it can write PC.
    if (g == 0b100) {
        bool L = (op >> 20) & 1;
        if (L && (op & (1u << 15))) return true;  // LDM with PC in list
    }

    // Data-processing (immediate or register form, bits 27..26 = 00).
    // Check Rd (bits 15..12); writing PC = branch.
    if (g == 0b000 || g == 0b001) {
        // Exclude MUL/MLA encoding (bits 27..22 = 0, bits 7..4 = 1001).
        // MUL has its own Rd field at bits 19..16 -- if that's R15 it's
        // also a PC-write, but MUL/Rd=15 is very rare and the handler
        // would catch it. We mark Rd=15 in data-proc only.
        uint8_t rd = (op >> 12) & 0xF;
        if (rd == 15) return true;
    }

    // Single data transfer (LDR/STR immediate or register): bits
    // 27..26 = 01. Rt at bits 15..12; load into PC = branch.
    if ((op >> 26 & 0x3) == 0b01) {
        bool L = (op >> 20) & 1;
        uint8_t rt = (op >> 12) & 0xF;
        if (L && rt == 15) return true;
    }

    return false;
}

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

    for (uint32_t i = 0; i < ARM_BLOCK_DIR_SIZE; i++) {
        arm_block_dir[i].start_pc     = 0xFFFFFFFFu;
        arm_block_dir[i].length       = 0;
        arm_block_dir[i].total_cycles = 0;
        arm_block_dir[i].ops_offset   = 0;
        arm_block_dir[i].generation   = 0;
        arm_block_dir[i].page_idx     = ARM_PAGE_NONE;
        arm_block_dir[i].page_gen     = 0;
        arm_block_dir[i].native_entry = NULL;
    }

    arm_uop_pool_head      = 0;
    arm_block_current_gen  = 1;
    arm_block_enabled      = true;
    return true;
}

void arm_block_uninit(void) {
    arm_block_enabled = false;
    if (arm_block_dir) { kfree(arm_block_dir); arm_block_dir = NULL; }
    if (arm_uop_pool)  { kfree(arm_uop_pool);  arm_uop_pool  = NULL; }
}

__attribute__((noinline))
const arm_block_t *arm_block_lookup(uint32_t inst_pc) {
    if (!arm_block_enabled) return NULL;
    uint8_t reg = (inst_pc >> 24) & 0xFu;
    bool is_rom = (reg >= 0x8 && reg <= 0xD);
    bool is_ram = (reg == 0x02 || reg == 0x03);
    if (!is_rom && !is_ram) return NULL;

    arm_block_t *b = &arm_block_dir[arm_block_hash(inst_pc)];
    if (b->start_pc != inst_pc) return NULL;
    if (b->generation != arm_block_current_gen) return NULL;
    if (b->page_idx != ARM_PAGE_NONE) {
        if (thumb_page_gen[b->page_idx] != b->page_gen) return NULL;
    }
    return b;
}

static inline uint32_t decode_read_op32(uint32_t pc) {
    uint8_t reg = (pc >> 24) & 0xFu;
    if (reg >= 0x8 && reg <= 0xD) {
        return rom_buffer_read_32_fast(&rom_buffer, pc);
    }
    if (reg == 0x03) {
        return mem_swz_read_w(wram_chip, pc & 0x7FFFu);
    }
    if (reg == 0x02) {
        return mem_swz_read_w(wram_board, pc & ewram_mask);
    }
    return 0;
}

__attribute__((noinline))
const arm_block_t *arm_block_decode(uint32_t inst_pc) {
    if (!arm_block_enabled) return NULL;
    uint8_t reg = (inst_pc >> 24) & 0xFu;
    bool is_rom = (reg >= 0x8 && reg <= 0xD);
    bool is_ram = (reg == 0x02 || reg == 0x03);
    if (!is_rom && !is_ram) return NULL;

    uint16_t page_idx = ARM_PAGE_NONE;
    uint16_t page_gen = 0;
    uint32_t page_end_pc = 0xFFFFFFFFu;
    if (is_ram) {
        page_idx = ram_page_idx(inst_pc);
        if (page_idx == ARM_PAGE_NONE) return NULL;
        page_gen = thumb_page_gen[page_idx];
        page_end_pc = (inst_pc | ARM_PAGE_MASK) + 1u;
    }

    if (arm_uop_pool_head + ARM_BLOCK_MAX_LEN > ARM_UOP_POOL_SIZE) {
        arm_uop_pool_head = 0;
        arm_block_current_gen++;
        if (arm_block_current_gen == 0) arm_block_current_gen = 1;
    }

    arm_uop_t *ops = arm_uop_pool + arm_uop_pool_head;
    uint32_t pc = inst_pc;
    uint16_t length = 0;
    uint16_t total_cycles = 0;
    uint8_t  region_ws;
    if (is_rom) region_ws = ws_s_arm[(pc >> 25) & 3];
    else if (reg == 0x03) region_ws = 1;
    else                  region_ws = 6;     // EWRAM word access is 2x t16

    while (length < ARM_BLOCK_MAX_LEN && pc < page_end_pc) {
        uint8_t r = (pc >> 24) & 0xFu;
        bool r_rom = (r >= 0x8 && r <= 0xD);
        bool r_ram = (r == 0x02 || r == 0x03);
        if (is_rom && !r_rom) break;
        if (is_ram && !r_ram) break;

        uint32_t op = decode_read_op32(pc);

        decode_arm_op(op, &ops[length]);
        ops[length].cycles = region_ws;

        total_cycles += region_ws;
        length++;

        if (arm_block_ends(op)) break;

        pc += 4;
        if (is_rom) region_ws = ws_s_arm[(pc >> 25) & 3];
    }

    if (length == 0) return NULL;

    arm_block_t *slot = &arm_block_dir[arm_block_hash(inst_pc)];
    slot->start_pc     = inst_pc;
    slot->length       = length;
    slot->total_cycles = total_cycles;
    slot->ops_offset   = (uint16_t)arm_uop_pool_head;
    slot->generation   = arm_block_current_gen;
    slot->page_idx     = page_idx;
    slot->page_gen     = page_gen;
    // Decoder default: NULL native_entry so the executor takes the
    // interpreter path. Explicit because we may be overwriting an entry
    // from a different PC that hashed here.
    slot->native_entry = NULL;

    arm_uop_pool_head += length;

#if GBA_JIT_ARM
    // Phase 1 Chunk 4: try to JIT-compile the block. On any failure
    // (arena full, block too long, etc.) native_entry stays NULL and
    // the executor falls back to walking the uop list.
    (void)arm_jit_compile_block(slot);
#endif
    return slot;
}
