#include <stdlib.h>
#include <string.h>  // Add this for memset

#include <gint/defs/attributes.h>

#include "arm.h"
#include "arm_mem.h"
#include "arm_shared.h"

#include "io.h"
#include "timer.h"

// EXTERN VARIABLES DECLARATION ===

// Hot interpreter state placed in XYRAM (16 KB on-chip SRAM, 1-cycle
// deterministic access regardless of cache state). Preserved across
// gint_world_switch via the backup buffer set up in main(). Total
// footprint: arm_r (~256B) + dispatch state (~36B) = ~292 bytes.

arm_regs_t arm_r GXRAM;

uint32_t arm_op GXRAM;
uint32_t arm_pipe[2] GXRAM;
uint32_t arm_cycles GXRAM;

bool int_halt GXRAM;
bool pipe_reload GXRAM;

// Lazy NZCV. Authoritative inside the inner exec loop; cpsr's top nibble is
// only synced at boundaries where guest code or the host can observe it
// (SPSR save/restore, MSR/MRS, arm_int, diagnostic dumps).
// Each holds 0 or 1.
uint32_t arm_flag_n GXRAM;
uint32_t arm_flag_z GXRAM;
uint32_t arm_flag_c GXRAM;
uint32_t arm_flag_v GXRAM;

//=================================

/*
 * Utils
 */

static void arm_flag_set(uint32_t flag, bool cond) {
    if (cond)
        arm_r.cpsr |= flag;
    else
        arm_r.cpsr &= ~flag;
}

// Pack lazy NZCV into cpsr. Call before any code reads cpsr's top nibble.
void arm_flags_to_cpsr(void) {
    uint32_t nzcv = (arm_flag_n << 31)
                  | (arm_flag_z << 30)
                  | (arm_flag_c << 29)
                  | (arm_flag_v << 28);
    arm_r.cpsr = (arm_r.cpsr & 0x0fffffff) | nzcv;
}

// Unpack cpsr's top nibble into lazy NZCV. Call after any write that may
// have changed cpsr's top nibble (SPSR restore, MSR with mask covering top byte).
static inline void arm_flags_from_cpsr(void) {
    arm_flag_n = (arm_r.cpsr >> 31) & 1;
    arm_flag_z = (arm_r.cpsr >> 30) & 1;
    arm_flag_c = (arm_r.cpsr >> 29) & 1;
    arm_flag_v = (arm_r.cpsr >> 28) & 1;
}

static void arm_bank_to_regs(int8_t mode) {
    if (mode != ARM_FIQ) {
        arm_r.r[8]  = arm_r.r8_usr;
        arm_r.r[9]  = arm_r.r9_usr;
        arm_r.r[10] = arm_r.r10_usr;
        arm_r.r[11] = arm_r.r11_usr;
        arm_r.r[12] = arm_r.r12_usr;
    }

    switch (mode) {
        case ARM_USR:
        case ARM_SYS:
            arm_r.r[13] = arm_r.r13_usr;
            arm_r.r[14] = arm_r.r14_usr;
        break;

        case ARM_FIQ:
            arm_r.r[8]  = arm_r.r8_fiq;
            arm_r.r[9]  = arm_r.r9_fiq;
            arm_r.r[10] = arm_r.r10_fiq;
            arm_r.r[11] = arm_r.r11_fiq;
            arm_r.r[12] = arm_r.r12_fiq;
            arm_r.r[13] = arm_r.r13_fiq;
            arm_r.r[14] = arm_r.r14_fiq;
        break;

        case ARM_IRQ:
            arm_r.r[13] = arm_r.r13_irq;
            arm_r.r[14] = arm_r.r14_irq;
        break;

        case ARM_SVC:
            arm_r.r[13] = arm_r.r13_svc;
            arm_r.r[14] = arm_r.r14_svc;
        break;

        case ARM_MON:
            arm_r.r[13] = arm_r.r13_mon;
            arm_r.r[14] = arm_r.r14_mon;
        break;

        case ARM_ABT:
            arm_r.r[13] = arm_r.r13_abt;
            arm_r.r[14] = arm_r.r14_abt;
        break;

        case ARM_UND:
            arm_r.r[13] = arm_r.r13_und;
            arm_r.r[14] = arm_r.r14_und;
        break;
    }
}

static void arm_regs_to_bank(int8_t mode) {
    if (mode != ARM_FIQ) {
        arm_r.r8_usr  = arm_r.r[8];
        arm_r.r9_usr  = arm_r.r[9];
        arm_r.r10_usr = arm_r.r[10];
        arm_r.r11_usr = arm_r.r[11];
        arm_r.r12_usr = arm_r.r[12];
    }

    switch (mode) {
        case ARM_USR:
        case ARM_SYS:
            arm_r.r13_usr = arm_r.r[13];
            arm_r.r14_usr = arm_r.r[14];
        break;

        case ARM_FIQ:
            arm_r.r8_fiq  = arm_r.r[8];
            arm_r.r9_fiq  = arm_r.r[9];
            arm_r.r10_fiq = arm_r.r[10];
            arm_r.r11_fiq = arm_r.r[11];
            arm_r.r12_fiq = arm_r.r[12];
            arm_r.r13_fiq = arm_r.r[13];
            arm_r.r14_fiq = arm_r.r[14];
        break;

        case ARM_IRQ:
            arm_r.r13_irq = arm_r.r[13];
            arm_r.r14_irq = arm_r.r[14];
        break;

        case ARM_SVC:
            arm_r.r13_svc = arm_r.r[13];
            arm_r.r14_svc = arm_r.r[14];
        break;

        case ARM_MON:
            arm_r.r13_mon = arm_r.r[13];
            arm_r.r14_mon = arm_r.r[14];
        break;

        case ARM_ABT:
            arm_r.r13_abt = arm_r.r[13];
            arm_r.r14_abt = arm_r.r[14];
        break;

        case ARM_UND:
            arm_r.r13_und = arm_r.r[13];
            arm_r.r14_und = arm_r.r[14];
        break;
    }
}

static void arm_mode_set(int8_t mode) {
    int8_t curr = arm_r.cpsr & 0x1f;

    arm_r.cpsr &= ~0x1f;
    arm_r.cpsr |= mode;

    arm_regs_to_bank(curr);
    arm_bank_to_regs(mode);
}

static inline bool arm_flag_tst(uint32_t flag) {
    return arm_r.cpsr & flag;
}

static inline bool arm_cond(int8_t cond) {
    bool res;

    bool n = arm_flag_n;
    bool z = arm_flag_z;
    bool c = arm_flag_c;
    bool v = arm_flag_v;

    switch (cond >> 1) {
        case 0: res = z; break;
        case 1: res = c; break;
        case 2: res = n; break;
        case 3: res = v; break;
        case 4: res = c && !z; break;
        case 5: res = n == v; break;
        case 6: res = !z && n == v; break;
        default: res = true; break;
    }

    if (cond & 1) res = !res;

    return res;
}

static void arm_spsr_get(uint32_t *psr) {
    switch (arm_r.cpsr & 0x1f) {
        case ARM_FIQ: *psr = arm_r.spsr_fiq; break;
        case ARM_IRQ: *psr = arm_r.spsr_irq; break;
        case ARM_SVC: *psr = arm_r.spsr_svc; break;
        case ARM_MON: *psr = arm_r.spsr_mon; break;
        case ARM_ABT: *psr = arm_r.spsr_abt; break;
        case ARM_UND: *psr = arm_r.spsr_und; break;
    }
}

static void arm_spsr_set(uint32_t spsr) {
    switch (arm_r.cpsr & 0x1f) {
        case ARM_FIQ: arm_r.spsr_fiq = spsr; break;
        case ARM_IRQ: arm_r.spsr_irq = spsr; break;
        case ARM_SVC: arm_r.spsr_svc = spsr; break;
        case ARM_MON: arm_r.spsr_mon = spsr; break;
        case ARM_ABT: arm_r.spsr_abt = spsr; break;
        case ARM_UND: arm_r.spsr_und = spsr; break;
    }
}

static void arm_spsr_to_cpsr() {
    int8_t curr = arm_r.cpsr & 0x1f;

    arm_spsr_get(&arm_r.cpsr);

    int8_t mode = arm_r.cpsr & 0x1f;

    arm_regs_to_bank(curr);
    arm_bank_to_regs(mode);

    // SPSR's NZCV just landed in cpsr; resync lazy fields.
    arm_flags_from_cpsr();
}

static inline void arm_setn(uint32_t res) {
    arm_flag_n = res >> 31;
}

static inline void arm_setn64(uint64_t res) {
    arm_flag_n = (uint32_t)(res >> 63);
}

static inline void arm_setz(uint32_t res) {
    arm_flag_z = (res == 0);
}

static inline void arm_setz64(uint64_t res) {
    arm_flag_z = (res == 0);
}

static inline void arm_addc(uint64_t res) {
    arm_flag_c = (res > 0xffffffffULL);
}

static inline void arm_subc(uint64_t res) {
    arm_flag_c = (res < 0x100000000ULL);
}

static inline void arm_addv(uint32_t lhs, uint32_t rhs, uint32_t res) {
    arm_flag_v = ((~(lhs ^ rhs) & (lhs ^ res)) >> 31) & 1;
}

static inline void arm_subv(uint32_t lhs, uint32_t rhs, uint32_t res) {
    arm_flag_v = (((lhs ^ rhs) & (lhs ^ res)) >> 31) & 1;
}

static uint32_t arm_saturate(int64_t val, int32_t min, int32_t max, bool q) {
    uint32_t res = (uint32_t)val;

    if (val > max || val < min) {
        if (val > max)
            res = (uint32_t)max;
        else
            res = (uint32_t)min;

        if (q) arm_flag_set(ARM_Q, true);
    }

    return res;
}

static uint32_t arm_ssatq(int64_t val) {
    return arm_saturate(val, 0x80000000, 0x7fffffff, true);
}

static inline bool arm_in_thumb() {
    return arm_flag_tst(ARM_T);
}

// Read 16/32-bit little-endian values from a byte buffer. The GBA stores
// halfwords/words in little-endian byte order; on a big-endian host (SH4 on
// the CG50) a native `*(uint16_t *)` / `*(uint32_t *)` cast would read in
// big-endian byte order and decode bogus opcodes.
static inline uint16_t le16_at(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline uint32_t le32_at(const uint8_t *p) {
    return  (uint32_t)p[0]        |
           ((uint32_t)p[1] <<  8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static uint16_t arm_fetchh(access_type_e at) {
    switch (arm_r.r[15] >> 24) {
        case 0x0: arm_cycles += 1; bios_op = le32_at(bios + (arm_r.r[15] & 0x3ffc)); return le16_at(bios + (arm_r.r[15] & 0x3fff));
        case 0x2: arm_cycles += 3; return le16_at(wram_board + (arm_r.r[15] & ewram_mask));
        case 0x3: arm_cycles += 1; return le16_at(wram_chip + (arm_r.r[15] & 0x7fff));
        case 0x5: arm_cycles += 1; return le16_at(palette_ram + (arm_r.r[15] & 0x3ff));
        case 0x7: arm_cycles += 1; return le16_at(oam + (arm_r.r[15] & 0x3ff));

        case 0x8:
        case 0x9:
            if (at == NON_SEQ)
                arm_cycles += ws_n_t16[0];
            else
                arm_cycles += ws_s_t16[0];

            return rom_buffer_read_16_fast(&rom_buffer, arm_r.r[15]);

        case 0xa:
        case 0xb:
            if (at == NON_SEQ)
                arm_cycles += ws_n_t16[1];
            else
                arm_cycles += ws_s_t16[1];

            return rom_buffer_read_16_fast(&rom_buffer, arm_r.r[15]);

        case 0xc:
        case 0xd:
            if (at == NON_SEQ)
                arm_cycles += ws_n_t16[2];
            else
                arm_cycles += ws_s_t16[2];

            return rom_buffer_read_16_fast(&rom_buffer, arm_r.r[15]);

        default:
            if (at == NON_SEQ)
                return arm_readh_n(arm_r.r[15]);
            else
                return arm_readh_s(arm_r.r[15]);
    }
}

static uint32_t arm_fetch(access_type_e at) {
    switch (arm_r.r[15] >> 24) {
        case 0x0: arm_cycles += 1; return (bios_op = le32_at(bios + (arm_r.r[15] & 0x3ffc)));
        case 0x2: arm_cycles += 6; return le32_at(wram_board + (arm_r.r[15] & (ewram_mask & ~3u)));
        case 0x3: arm_cycles += 1; return le32_at(wram_chip + (arm_r.r[15] & 0x7ffc));
        case 0x5: arm_cycles += 1; return le32_at(palette_ram + (arm_r.r[15] & 0x3fc));
        case 0x7: arm_cycles += 1; return le32_at(oam + (arm_r.r[15] & 0x3fc));

        case 0x8:
        case 0x9:
            if (at == NON_SEQ)
                arm_cycles += ws_n_arm[0];
            else
                arm_cycles += ws_s_arm[0];

            return rom_buffer_read_32_fast(&rom_buffer, arm_r.r[15]);

        case 0xa:
        case 0xb:
            if (at == NON_SEQ)
                arm_cycles += ws_n_arm[1];
            else
                arm_cycles += ws_s_arm[1];

            return rom_buffer_read_32_fast(&rom_buffer, arm_r.r[15]);

        case 0xc:
        case 0xd:
            if (at == NON_SEQ)
                arm_cycles += ws_n_arm[2];
            else
                arm_cycles += ws_s_arm[2];

            return rom_buffer_read_32_fast(&rom_buffer, arm_r.r[15]);

        default:
            if (at == NON_SEQ)
                return arm_read_n(arm_r.r[15]);
            else
                return arm_read_s(arm_r.r[15]);
    }
}

static uint32_t arm_fetch_n() {
    uint32_t op;

    if (arm_in_thumb()) {
        op = arm_fetchh(NON_SEQ);

        arm_r.r[15] += 2;
    } else {
        op = arm_fetch(NON_SEQ);

        arm_r.r[15] += 4;
    }

    return op;
}

static uint32_t arm_fetch_s() {
    uint32_t op;

    if (arm_in_thumb()) {
        op = arm_fetchh(SEQUENTIAL);

        arm_r.r[15] += 2;
    } else {
        op = arm_fetch(SEQUENTIAL);

        arm_r.r[15] += 4;
    }

    return op;
}

extern volatile uint32_t arm_first_low_pc;
extern volatile uint32_t arm_first_low_lr;
extern volatile uint32_t arm_first_low_op;
extern volatile uint8_t  arm_first_low_was_thumb;
extern volatile uint8_t  arm_first_low_set;

// Sentinel address used to mark a return-from-HLE'd-IRQ. The user IRQ
// handler is invoked with LR = HLE_IRQ_RETURN_PC; when it does BX LR
// (or any other return form), PC becomes the sentinel and arm_load_pipe()
// runs the BIOS post-handler equivalent in place of fetching at the
// sentinel address. Chosen at 0xFFFF0000: region 0xFF is unmapped on
// the GBA, so no real cart code or data can ever legitimately land here.
#define HLE_IRQ_RETURN_PC  0xFFFF0000u

// Forward declaration; definition is colocated with the rest of the IRQ
// HLE near hle_swi() further down.
static void hle_irq_return(void);

// SH4A prefetch hint: non-blocking line fill into the operand cache.
// `pref @Rn` triggers an async fetch and returns immediately; if the line
// is already cached or the address is unmapped, it's a no-op. We use it
// to warm the cache line just past the basic block we're about to step
// through, hiding extram-chunk-buffer load latency behind the dispatch
// itself.
static inline void sh4_pref(const void *p) {
    __asm__ volatile ("pref @%0" : : "r" (p));
}

static void arm_load_pipe() {
    // Detect return from an HLE'd IRQ before attempting any fetch. The
    // sentinel address is unmapped; calling arm_fetch_n at sentinel would
    // hit the slow path and return garbage. hle_irq_return restores the
    // saved registers + CPSR and overwrites arm_r.r[15] with the real
    // return address, then we fall through to the normal fetch logic.
    if (arm_r.r[15] == HLE_IRQ_RETURN_PC) {
        hle_irq_return();
        // hle_irq_return updates arm_r.r[15] (and ARM_T flag) to the
        // real return address. Fall through to fetch from there.
    }

#ifdef ARM_TRACE_ENABLE
    // Tripwire: first time PC drops below 0x4000 from any branch. Used in
    // the past to find spurious entries into BIOS region. Costs a compare
    // and a load per pipeline reload, which is per-branch-not-per-step but
    // still adds up. Compile with -DARM_TRACE_ENABLE to opt in.
    if (!arm_first_low_set && arm_r.r[15] < 0x00004000) {
        arm_first_low_pc = arm_r.r[15];
        arm_first_low_lr = arm_r.r[14];
        arm_first_low_op = arm_op;
        arm_first_low_was_thumb = arm_in_thumb() ? 1 : 0;
        arm_first_low_set = 1;
    }
#endif

    arm_pipe[0] = arm_fetch_n();
    arm_pipe[1] = arm_fetch_s();

    // Prefetch the next 32-byte line of cart ROM. Per-branch overhead is
    // negligible; the line will be hot when t16_step / arm_step starts
    // walking the basic block. Only fire when the target is in the cart
    // ROM regions (where chunk-buffer reads pay extram bus latency)
    // AND the line stays inside the currently-pinned chunk — straddling
    // would prefetch the wrong physical address (the chunk_buffer of the
    // *current* chunk, not the one that holds the straddled bytes).
    {
        uint32_t pc8 = arm_r.r[15] >> 24;
        if (pc8 >= 0x8 && pc8 <= 0xd && rom_buffer.last_chunk_idx >= 0) {
            uint32_t off = arm_r.r[15] & (ROM_CHUNK_SIZE - 1);
            if (off < ROM_CHUNK_SIZE - 64) {
                sh4_pref(rom_buffer.chunk_buffers[rom_buffer.last_chunk_idx]
                         + ((off + 32) & ~31u));
            }
        }
    }

    pipe_reload = true;
}

static void arm_r15_align() {
    if (arm_in_thumb())
        arm_r.r[15] &= ~1;
    else
        arm_r.r[15] &= ~3;
}

static void arm_interwork() {
    arm_flag_set(ARM_T, arm_r.r[15] & 1);

    arm_r15_align();
    arm_load_pipe();
}

static void arm_cycles_s_to_n() {
    if (arm_r.r[15] & 0x08000000) {
        uint8_t idx = (arm_r.r[15] >> 25) & 3;

        if (arm_in_thumb()) {
            arm_cycles -= ws_s_t16[idx];
            arm_cycles += ws_n_t16[idx];
        } else {
            arm_cycles -= ws_s_arm[idx];
            arm_cycles += ws_n_arm[idx];
        }
    }
}

/*
 * Execute
 */

//Data Processing (Arithmetic)
typedef struct {
    uint64_t lhs;
    uint64_t rhs;
    uint8_t  rd;
    bool     cout;
    bool     s;
} arm_data_t;

#define ARM_ARITH_SUB  0
#define ARM_ARITH_ADD  1

#define ARM_ARITH_NO_C   0
#define ARM_ARITH_CARRY  1

#define ARM_ARITH_NO_REV   0
#define ARM_ARITH_REVERSE  1

static inline void arm_arith_set(arm_data_t op, uint64_t res, bool add) {
    if (op.rd == 15) {
        if (op.s) {
            arm_spsr_to_cpsr();
            arm_r15_align();
            arm_load_pipe();
            arm_check_irq();
        } else {
            arm_r15_align();
            arm_load_pipe();
        }
    } else if (op.s) {
        arm_setn(res);
        arm_setz(res);

        if (add) {
            arm_addc(res);
            arm_addv(op.lhs, op.rhs, res);
        } else {
            arm_subc(res);
            arm_subv(op.lhs, op.rhs, res);
        }
    }
}

static inline void arm_arith_add(arm_data_t op, bool adc) {
    uint64_t res  = op.lhs + op.rhs;
    if (adc) res += arm_flag_c;

    arm_r.r[op.rd] = res;

    arm_arith_set(op, res, ARM_ARITH_ADD);
}

static inline void arm_arith_subtract(arm_data_t op, bool sbc, bool rev) {
    if (rev) {
        uint64_t tmp = op.lhs;

        op.lhs = op.rhs;
        op.rhs = tmp;
    }

    uint64_t res  = op.lhs - op.rhs;
    if (sbc) res -= !arm_flag_c;

    arm_r.r[op.rd] = res;

    arm_arith_set(op, res, ARM_ARITH_SUB);
}

static void arm_arith_rsb(arm_data_t op) {
    arm_arith_subtract(op, ARM_ARITH_NO_C, ARM_ARITH_REVERSE);
}

static void arm_arith_rsc(arm_data_t op) {
    arm_arith_subtract(op, ARM_ARITH_CARRY, ARM_ARITH_REVERSE);
}

static void arm_arith_sbc(arm_data_t op) {
    arm_arith_subtract(op, ARM_ARITH_CARRY, ARM_ARITH_NO_REV);
}

static inline void arm_arith_sub(arm_data_t op) {
    arm_arith_subtract(op, ARM_ARITH_NO_C, ARM_ARITH_NO_REV);
}

static inline void arm_arith_cmn(arm_data_t op) {
    arm_arith_set(op, op.lhs + op.rhs, ARM_ARITH_ADD);
}

static inline void arm_arith_cmp(arm_data_t op) {
    arm_arith_set(op, op.lhs - op.rhs, ARM_ARITH_SUB);
}

//Data Processing (Logical)
typedef enum {
    AND,
    BIC,
    EOR,
    MVN,
    ORN,
    ORR,
    SHIFT
} arm_logic_e;

static void arm_logic_set(arm_data_t op, uint32_t res) {
    if (op.rd == 15) {
        if (op.s) {
            arm_spsr_to_cpsr();
            arm_r15_align();
            arm_load_pipe();
            arm_check_irq();
        } else {
            arm_r15_align();
            arm_load_pipe();
        }
    } else if (op.s) {
        arm_setn(res);
        arm_setz(res);

        arm_flag_c = op.cout;
    }
}

static void arm_logic(arm_data_t op, arm_logic_e inst) {
    uint32_t res;

    switch (inst) {
        case AND: res = op.lhs &  op.rhs; break;
        case BIC: res = op.lhs & ~op.rhs; break;
        case EOR: res = op.lhs ^  op.rhs; break;
        case ORN: res = op.lhs | ~op.rhs; break;
        case ORR: res = op.lhs |  op.rhs; break;

        case MVN:   res = ~op.rhs; break;
        case SHIFT: res =  op.rhs; break;
    }

    arm_r.r[op.rd] = res;

    arm_logic_set(op, res);
}

static inline void arm_asr(arm_data_t op) {
    int64_t val = (int32_t)op.lhs;
    uint8_t sh = op.rhs;

    if (sh > 32) sh = 32;

    op.rhs = val >> sh;
    op.cout = val & (1 << (sh - 1));

    arm_r.r[op.rd] = op.rhs;

    arm_logic_set(op, op.rhs);
}

static inline void arm_lsl(arm_data_t op) {
    uint64_t val = op.lhs;
    uint8_t sh = op.rhs;

    bool c = arm_flag_c;

    op.rhs = val;
    op.cout = c;

    if (sh > 32) {
        op.rhs = 0;
        op.cout = 0;
    } else if (sh) {
        op.rhs = val << sh;
        op.cout = val & (1 << (32 - sh));
    }

    arm_r.r[op.rd] = op.rhs;

    arm_logic_set(op, op.rhs);
}

static inline void arm_lsr(arm_data_t op) {
    uint64_t val = op.lhs;
    uint8_t sh = op.rhs;

    bool c = arm_flag_c;

    op.rhs = val;
    op.cout = c;

    if (sh > 32) {
        op.rhs = 0;
        op.cout = 0;
    } else if (sh) {
        op.rhs = val >> sh;
        op.cout = val & (1 << (sh - 1));
    }

    arm_r.r[op.rd] = op.rhs;

    arm_logic_set(op, op.rhs);
}

static void arm_ror(arm_data_t op) {
    uint64_t val = op.lhs;
    uint8_t sh = op.rhs;

    bool c = arm_flag_c;

    op.rhs = val;
    op.cout = c;

    if (sh) {
        sh &= 0x1f;
        if (sh == 0) sh = 32;

        op.rhs = ROR(val, sh);
        op.cout = val & (1 << (sh - 1));
    }

    arm_r.r[op.rd] = op.rhs;

    arm_logic_set(op, op.rhs);
}

static void arm_count_zeros(uint8_t rd) {
    uint8_t rm = arm_op & 0xf;
    uint32_t m = arm_r.r[rm];
    uint32_t i, cnt = 32;

    for (i = 0; i < 32; i++) {
        if (m & (1 << i)) cnt = i ^ 0x1f;
    }

    arm_r.r[rd] = cnt;
}

static void arm_logic_teq(arm_data_t op) {
    arm_logic_set(op, op.lhs ^ op.rhs);
}

static void arm_logic_tst(arm_data_t op) {
    arm_logic_set(op, op.lhs & op.rhs);
}

//Data Processing Operand Decoding
typedef struct {
    uint32_t val;
    bool     cout;
} arm_shifter_t;

#define ARM_FLAG_KEEP  0
#define ARM_FLAG_SET   1

#define ARM_SHIFT_LEFT   0
#define ARM_SHIFT_RIGHT  1

static arm_data_t arm_data_imm_op() {
    uint32_t imm   = (arm_op >>  0) & 0xff;
    uint8_t  shift = (arm_op >>  7) & 0x1e;
    uint8_t  rd    = (arm_op >> 12) & 0xf;
    uint8_t  rn    = (arm_op >> 16) & 0xf;

    imm = ROR(imm, shift);

    arm_data_t op = {
        .rd   = rd,
        .lhs  = arm_r.r[rn],
        .rhs  = imm,
        .cout = imm & (1 << 31),
        .s    = SBIT(arm_op)
    };

    return op;
}

static arm_shifter_t arm_data_regi(uint8_t rm, uint8_t type, uint8_t imm) {
    arm_shifter_t out;

    out.val = arm_r.r[rm];
    out.cout = arm_flag_c;

    uint32_t m = out.val;
    uint8_t c = out.cout;

    switch (type) {
        case 0: //LSL
            if (imm) {
                out.val = m << imm;
                out.cout = m & (1 << (32 - imm));
            }
            break;

        case 1: //LSR
        case 2: //ASR
            if (imm == 0) imm = 32;

            if (type == 2)
                out.val = (int64_t)((int32_t)m) >> imm;
            else
                out.val = (uint64_t)m >> imm;

            out.cout = m & (1 << (imm - 1));
            break;

        case 3: //ROR
            if (imm) {
                out.val = ROR(m, imm);
                out.cout = m & (1 << (imm - 1));
            } else { //RRX
                out.val = ROR((m & ~1) | c, 1);
                out.cout = m & 1;
            }
            break;
    }

    return out;
}

static arm_data_t arm_data_regi_op() {
    uint8_t rm   = (arm_op >>  0) & 0xf;
    uint8_t type = (arm_op >>  5) & 0x3;
    uint8_t imm  = (arm_op >>  7) & 0x1f;
    uint8_t rd   = (arm_op >> 12) & 0xf;
    uint8_t rn   = (arm_op >> 16) & 0xf;

    arm_shifter_t shift = arm_data_regi(rm, type, imm);

    arm_data_t op = {
        .rd   = rd,
        .lhs  = arm_r.r[rn],
        .rhs  = shift.val,
        .cout = shift.cout,
        .s    = SBIT(arm_op)
    };

    return op;
}

static arm_shifter_t arm_data_regr(uint8_t rm, uint8_t type, uint8_t rs) {
    arm_shifter_t out;

    out.val = arm_r.r[rm];
    out.cout = arm_flag_c;

    if (rm == 15) out.val += 4;

    uint8_t sh = arm_r.r[rs];

    if (sh >= 32) {
        if (type == 3) sh &= 0x1f;
        if (type == 2 || sh == 0) sh = 32;

        out.val = 0;
        out.cout = 0;
    }

    if (sh && sh <= 32) {
        uint32_t m = arm_r.r[rm];

        switch (type) {
            case 0: //LSL
                out.val = (uint64_t)m << sh;
                out.cout = m & (1 << (32 - sh));
                break;

            case 1: //LSR
            case 2: //ASR
                if (type == 2)
                    out.val = (int64_t)((int32_t)m) >> sh;
                else
                    out.val = (uint64_t)m >> sh;

                out.cout = m & (1 << (sh - 1));
                break;

            case 3: //ROR
                out.val = ROR((uint64_t)m, sh);
                out.cout = m & (1 << (sh - 1));
                break;
        }
    }

    arm_cycles++;

    arm_cycles_s_to_n();

    return out;
}

static arm_data_t arm_data_regr_op() {
    uint8_t rm   = (arm_op >>  0) & 0xf;
    uint8_t type = (arm_op >>  5) & 0x3;
    uint8_t rs   = (arm_op >>  8) & 0xf;
    uint8_t rd   = (arm_op >> 12) & 0xf;
    uint8_t rn   = (arm_op >> 16) & 0xf;

    arm_shifter_t shift = arm_data_regr(rm, type, rs);

    arm_data_t data = {
        .rd   = rd,
        .lhs  = arm_r.r[rn],
        .rhs  = shift.val,
        .cout = shift.cout,
        .s    = SBIT(arm_op)
    };

    return data;
}

static arm_data_t t16_data_imm3_op() {
    uint8_t rd  = (arm_op >> 0) & 0x7;
    uint8_t rn  = (arm_op >> 3) & 0x7;
    uint8_t imm = (arm_op >> 6) & 0x7;

    arm_data_t op  = {
        .rd   = rd,
        .lhs  = arm_r.r[rn],
        .rhs  = imm,
        .cout = false,
        .s    = true
    };

    return op;
}

static inline arm_data_t t16_data_imm8_op() {
    uint8_t imm = (arm_op >> 0) & 0xff;
    uint8_t rdn = (arm_op >> 8) & 0x7;

    arm_data_t op = {
        .rd   = rdn,
        .lhs  = arm_r.r[rdn],
        .rhs  = imm,
        .cout = false,
        .s    = true
    };

    return op;
}

static arm_data_t t16_data_rdn3_op() {
    uint8_t rdn = (arm_op >> 0) & 7;
    uint8_t rm  = (arm_op >> 3) & 7;

    arm_data_t op = {
        .rd   = rdn,
        .lhs  = arm_r.r[rdn],
        .rhs  = arm_r.r[rm],
        .cout = false,
        .s    = true
    };

    return op;
}

static arm_data_t t16_data_neg_op() {
    uint8_t rd = (arm_op >> 0) & 7;
    uint8_t rm = (arm_op >> 3) & 7;

    arm_data_t op = {
        .rd   = rd,
        .lhs  = 0,
        .rhs  = arm_r.r[rm],
        .cout = false,
        .s    = true
    };

    return op;
}

static arm_data_t t16_data_reg_op() {
    uint8_t rd = (arm_op >> 0) & 7;
    uint8_t rn = (arm_op >> 3) & 7;
    uint8_t rm = (arm_op >> 6) & 7;

    arm_data_t op = {
        .rd   = rd,
        .lhs  = arm_r.r[rn],
        .rhs  = arm_r.r[rm],
        .cout = false,
        .s    = true
    };

    return op;
}

static arm_data_t t16_data_rdn4_op(bool s) {
    uint8_t rm = (arm_op >> 3) & 0xf;

    uint8_t rdn;

    rdn  = (arm_op >> 0) & 0x7;
    rdn |= (arm_op >> 4) & 0x8;

    arm_data_t op = {
        .rd   = rdn,
        .lhs  = arm_r.r[rdn],
        .rhs  = arm_r.r[rm],
        .cout = false,
        .s    = s
    };

    return op;
}

static arm_data_t t16_data_imm7sp_op() {
    uint16_t imm = (arm_op & 0x7f) << 2;

    arm_data_t op = {
        .rd   = 13,
        .lhs  = arm_r.r[13],
        .rhs  = imm,
        .cout = false,
        .s    = true
    };

    return op;
}

static arm_data_t t16_data_imm8sp_op() {
    arm_data_t op = t16_data_imm8_op();

    op.lhs = arm_r.r[13];
    op.rhs <<= 2;
    op.s = false;

    return op;
}

static arm_data_t t16_data_imm8pc_op() {
    arm_data_t op = t16_data_imm8_op();

    op.lhs = arm_r.r[15] & ~3;
    op.rhs <<= 2;
    op.s = false;

    return op;
}

static arm_data_t t16_data_imm5_op(bool rsh) {
    uint8_t rd  = (arm_op >> 0) & 0x7;
    uint8_t rn  = (arm_op >> 3) & 0x7;
    uint8_t imm = (arm_op >> 6) & 0x1f;

    arm_data_t op = {
        .rd   = rd,
        .lhs  = arm_r.r[rn],
        .rhs  = (rsh && imm == 0 ? 32 : imm),
        .cout = false,
        .s    = true
    };

    return op;
}

//Multiplication
typedef struct {
    uint64_t lhs;
    uint64_t rhs;
    uint8_t  ra;
    uint8_t  rd;
    bool     s;
} arm_mpy_t;

#define ARM_MPY_SIGNED  0
#define ARM_MPY_UNSIGN  1

static void arm_mpy_inc_cycles(uint64_t rhs, bool u) {
    uint32_t m1 = rhs & 0xffffff00;
    uint32_t m2 = rhs & 0xffff0000;
    uint32_t m3 = rhs & 0xff000000;

    if      (m1 == 0 || (!u && m1 == 0xffffff00))
        arm_cycles += 1;
    else if (m2 == 0 || (!u && m2 == 0xffff0000))
        arm_cycles += 2;
    else if (m3 == 0 || (!u && m3 == 0xff000000))
        arm_cycles += 3;
    else
        arm_cycles += 4;

    arm_cycles_s_to_n();
}

static void arm_mpy_add(arm_mpy_t op) {
    uint32_t a = arm_r.r[op.ra];

    uint32_t res = a + op.lhs * op.rhs;

    arm_r.r[op.rd] = res;

    if (op.s) {
        arm_setn(res);
        arm_setz(res);
    }

    arm_mpy_inc_cycles(op.rhs, ARM_MPY_SIGNED);

    arm_cycles++;
}

static void arm_mpy(arm_mpy_t op) {
    uint32_t res = op.lhs * op.rhs;

    arm_r.r[op.rd] = res;

    if (op.s) {
        arm_setn(res);
        arm_setz(res);
    }

    arm_mpy_inc_cycles(op.rhs, ARM_MPY_SIGNED);
}

static void arm_mpy_smla__(arm_mpy_t op, bool m, bool n) {
    if (m) op.lhs >>= 16;
    if (n) op.rhs >>= 16;

    int64_t l = (int16_t)op.lhs;
    int64_t r = (int16_t)op.rhs;

    int64_t res = arm_r.r[op.ra] + l * r;

    arm_r.r[op.rd] = res;

    int32_t d = arm_r.r[op.rd];

    if (d != res) arm_flag_set(ARM_Q, true);

    arm_mpy_inc_cycles(op.rhs, ARM_MPY_SIGNED);

    arm_cycles++;
}

static void arm_mpy_smlal(arm_mpy_t op) {
    int64_t l = (int32_t)op.lhs;
    int64_t r = (int32_t)op.rhs;

    int64_t a = arm_r.r[op.ra];
    int64_t d = arm_r.r[op.rd];

    a |= d << 32;

    int64_t res = a + l * r;

    arm_r.r[op.ra] = res;
    arm_r.r[op.rd] = res >> 32;

    if (op.s) {
        arm_setn64(res);
        arm_setz64(res);
    }

    arm_mpy_inc_cycles(op.rhs, ARM_MPY_SIGNED);

    arm_cycles += 2;
}

static void arm_mpy_smlal__(arm_mpy_t op, bool m, bool n) {
    if (m) op.lhs >>= 16;
    if (n) op.rhs >>= 16;

    int64_t l = (int16_t)op.lhs;
    int64_t r = (int16_t)op.rhs;

    int64_t a = arm_r.r[op.ra];
    int64_t d = arm_r.r[op.rd];

    a |= d << 32;

    int64_t res = a + l * r;

    arm_r.r[op.ra] = res;
    arm_r.r[op.rd] = res >> 32;

    arm_mpy_inc_cycles(op.rhs, ARM_MPY_SIGNED);

    arm_cycles += 2;
}

static void arm_mpy_smlaw_(arm_mpy_t op, bool m) {
    if (m) op.lhs >>= 16;

    int64_t l = (int16_t)op.lhs;
    int64_t a = arm_r.r[op.ra];

    int64_t res = (a << 16) + l * op.rhs;

    arm_r.r[op.rd] = res >> 16;

    int32_t d = (int32_t)arm_r.r[op.rd];

    if (d != res) arm_flag_set(ARM_Q, true);

    arm_mpy_inc_cycles(op.rhs, ARM_MPY_SIGNED);

    arm_cycles++;
}

static void arm_mpy_smul(arm_mpy_t op, bool m, bool n) {
    if (m) op.lhs >>= 16;
    if (n) op.rhs >>= 16;

    int64_t l = (int16_t)op.lhs;
    int64_t r = (int16_t)op.rhs;

    int64_t res = l * r;

    arm_r.r[op.rd] = res;

    arm_mpy_inc_cycles(op.rhs, ARM_MPY_SIGNED);
}

static void arm_mpy_smull(arm_mpy_t op) {
    int64_t l = (int32_t)op.lhs;
    int64_t r = (int32_t)op.rhs;

    int64_t res = l * r;

    arm_r.r[op.ra] = res;
    arm_r.r[op.rd] = res >> 32;

    if (op.s) {
        arm_setn64(res);
        arm_setz64(res);
    }

    arm_mpy_inc_cycles(op.rhs, ARM_MPY_SIGNED);

    arm_cycles++;
}

static void arm_mpy_smulw_(arm_mpy_t op, bool m) {
    if (m) op.lhs >>= 16;

    int64_t res =  (int16_t)op.lhs * op.rhs;

    arm_r.r[op.rd] = res >> 16;

    arm_mpy_inc_cycles(op.rhs, ARM_MPY_SIGNED);
}

static void arm_mpy_umlal(arm_mpy_t op) {
    uint64_t l = op.lhs;
    uint64_t r = op.rhs;

    uint64_t a = arm_r.r[op.ra];
    uint64_t d = arm_r.r[op.rd];

    a |= d << 32;

    uint64_t res = a + l * r;

    arm_r.r[op.ra] = res;
    arm_r.r[op.rd] = res >> 32;

    if (op.s) {
        arm_setn64(res);
        arm_setz64(res);
    }

    arm_mpy_inc_cycles(op.rhs, ARM_MPY_UNSIGN);

    arm_cycles += 2;
}

static void arm_mpy_umull(arm_mpy_t op) {
    uint64_t l = op.lhs;
    uint64_t r = op.rhs;

    uint64_t res = l * r;

    arm_r.r[op.ra] = res;
    arm_r.r[op.rd] = res >> 32;

    if (op.s) {
        arm_setn64(res);
        arm_setz64(res);
    }

    arm_mpy_inc_cycles(op.rhs, ARM_MPY_UNSIGN);

    arm_cycles++;
}

static arm_mpy_t arm_mpy_op() {
    uint8_t rn = (arm_op >>  0) & 0xf;
    uint8_t rm = (arm_op >>  8) & 0xf;
    uint8_t ra = (arm_op >> 12) & 0xf;
    uint8_t rd = (arm_op >> 16) & 0xf;
    bool    s  = (arm_op >> 20) & 0x1;

    arm_mpy_t op = {
        .lhs = arm_r.r[rn],
        .rhs = arm_r.r[rm],
        .ra  = ra,
        .rd  = rd,
        .s   = s
    };

    return op;
}

static arm_mpy_t t16_mpy_op() {
    uint8_t rdn = (arm_op >> 0) & 7;
    uint8_t rm  = (arm_op >> 3) & 7;

    arm_mpy_t op = {
        .rd   = rdn,
        .lhs  = arm_r.r[rdn],
        .rhs  = arm_r.r[rm],
        .s    = true
    };

    return op;
}

//Processor State
#define PRIV_MASK   0xf8ff03df
#define USR_MASK    0xf8ff0000
#define STATE_MASK  0x01000020

typedef struct {
    uint8_t  rd;
    uint32_t psr;
    uint32_t mask;
    bool     r;
} arm_psr_t;

static void arm_psr_to_reg(arm_psr_t op) {
    if (op.r) {
        arm_spsr_get(&arm_r.r[op.rd]);
    } else {
        // MRS reads cpsr — make sure lazy NZCV is packed in first.
        arm_flags_to_cpsr();
        if ((arm_r.cpsr & 0x1f) == ARM_USR)
            arm_r.r[op.rd] = arm_r.cpsr & USR_MASK;
        else
            arm_r.r[op.rd] = arm_r.cpsr & PRIV_MASK;
    }
}

static void arm_reg_to_psr(arm_psr_t op) {
    uint32_t spsr = 0;
    uint32_t mask = 0;

    if (op.mask & 1) mask  = 0x000000ff;
    if (op.mask & 2) mask |= 0x0000ff00;
    if (op.mask & 4) mask |= 0x00ff0000;
    if (op.mask & 8) mask |= 0xff000000;

    uint32_t sec_msk;

    if ((arm_r.cpsr & 0x1f) == ARM_USR)
        sec_msk = USR_MASK;
    else
        sec_msk = PRIV_MASK;

    if (op.r) sec_msk |= STATE_MASK;

    mask &= sec_msk;

    op.psr &= mask;

    if (op.r) {
        arm_spsr_get(&spsr);
        arm_spsr_set((spsr & ~mask) | op.psr);
    } else {
        int8_t curr = arm_r.cpsr & 0x1f;
        int8_t mode = op.psr     & 0x1f;

        arm_r.cpsr &= ~mask;
        arm_r.cpsr |= op.psr;

        arm_regs_to_bank(curr);
        arm_bank_to_regs(mode);

        // MSR may have rewritten cpsr's NZCV bits; resync lazy fields.
        arm_flags_from_cpsr();

        arm_check_irq();
    }
}

static arm_psr_t arm_mrs_op() {
    arm_psr_t op = {
        .rd = (arm_op >> 12) & 0xf,
        .r  = (arm_op >> 22) & 0x1
    };

    return op;
}

static arm_psr_t arm_msr_imm_op() {
    arm_data_t data = arm_data_imm_op();

    arm_psr_t op = {
        .psr  = data.rhs,
        .mask = (arm_op >> 16) & 0xf,
        .r    = (arm_op >> 22) & 0x1
    };

    return op;
}

static arm_psr_t arm_msr_reg_op() {
    uint8_t rm   = (arm_op >>  0) & 0xf;
    uint8_t mask = (arm_op >> 16) & 0xf;
    bool    r    = (arm_op >> 22) & 0x1;

    arm_psr_t op = {
        .psr  = arm_r.r[rm],
        .mask = mask,
        .r    = r
    };

    return op;
}

//Load/Store
typedef struct {
    uint16_t regs;
    uint8_t  rn;
    uint8_t  rt2;
    uint8_t  rt;
    uint32_t addr;
    int32_t  disp;
} arm_memio_t;

typedef enum {
    BYTE  = 1,
    HWORD = 2,
    WORD  = 4,
    DWORD = 8
} arm_size_e;

static uint32_t arm_memio_reg_get(uint8_t reg) {
    if (reg == 15)
        return arm_r.r[15] + 4;
    else
        return arm_r.r[reg];
}

static void arm_memio_ldm(arm_memio_t op) {
    uint8_t i;

    arm_r.r[op.rn] += op.disp;

    for (i = 0; i < 16; i++) {
        if (op.regs & (1 << i)) {
            arm_r.r[i] = arm_read_s(op.addr);

            op.addr += 4;
        }
    }

    if (op.regs & 0x8000) {
        arm_r15_align();
        arm_load_pipe();
    }

    arm_cycles++;

    arm_cycles_s_to_n();
}

static void arm_memio_ldm_usr(arm_memio_t op) {
    if (arm_op & 0x8000) {
        arm_memio_ldm(op);
        arm_spsr_to_cpsr();
        arm_check_irq();
    } else {
        int8_t mode = arm_r.cpsr & 0x1f;

        arm_mode_set(ARM_USR);
        arm_memio_ldm(op);
        arm_mode_set(mode);
    }
}

static void arm_memio_stm(arm_memio_t op) {
    bool first = true;
    uint8_t i;

    for (i = 0; i < 16; i++) {
        if (op.regs & (1 << i)) {
            arm_write_s(op.addr, arm_memio_reg_get(i));

            if (first) {
                arm_r.r[op.rn] += op.disp;

                first = false;
            }

            op.addr += 4;
        }
    }

    arm_cycles_s_to_n();
}

static void arm_memio_stm_usr(arm_memio_t op) {
    int8_t mode = arm_r.cpsr & 0x1f;

    arm_mode_set(ARM_USR);
    arm_memio_stm(op);
    arm_mode_set(mode);
}

static inline void arm_memio_ldr(arm_memio_t op) {
    arm_r.r[op.rt] = arm_read_n(op.addr);

    if (op.rt == 15) {
        arm_r15_align();
        arm_load_pipe();
    }

    arm_cycles++;

    arm_cycles_s_to_n();
}

static inline void arm_memio_ldrb(arm_memio_t op) {
    arm_r.r[op.rt] = arm_readb_n(op.addr);

    if (op.rt == 15) {
        arm_r15_align();
        arm_load_pipe();
    }

    arm_cycles++;

    arm_cycles_s_to_n();
}

static inline void arm_memio_ldrh(arm_memio_t op) {
    arm_r.r[op.rt] = arm_readh_n(op.addr);

    if (op.rt == 15) {
        arm_r15_align();
        arm_load_pipe();
    }

    arm_cycles++;

    arm_cycles_s_to_n();
}

static void arm_memio_ldrsb(arm_memio_t op) {
    int32_t value = arm_readb_n(op.addr);

    value <<= 24;
    value >>= 24;

    arm_r.r[op.rt] = value;

    if (op.rt == 15) {
        arm_r15_align();
        arm_load_pipe();
    }

    arm_cycles++;

    arm_cycles_s_to_n();
}

static void arm_memio_ldrsh(arm_memio_t op) {
    int32_t value = arm_readh_n(op.addr);

    value <<= (op.addr & 1 ? 24 : 16);
    value >>= (op.addr & 1 ? 24 : 16);

    arm_r.r[op.rt] = value;

    if (op.rt == 15) {
        arm_r15_align();
        arm_load_pipe();
    }

    arm_cycles++;

    arm_cycles_s_to_n();
}

static void arm_memio_ldrd(arm_memio_t op) {
    arm_r.r[op.rt]  = arm_read_n(op.addr + 0);
    arm_r.r[op.rt2] = arm_read_s(op.addr + 4);

    if (op.rt2 == 15) {
        arm_r15_align();
        arm_load_pipe();
    }

    arm_cycles++;

    arm_cycles_s_to_n();
}

static inline void arm_memio_str(arm_memio_t op) {
    arm_write_n(op.addr, arm_memio_reg_get(op.rt));

    arm_cycles_s_to_n();
}

static inline void arm_memio_strb(arm_memio_t op) {
    arm_writeb_n(op.addr, arm_memio_reg_get(op.rt));

    arm_cycles_s_to_n();
}

static inline void arm_memio_strh(arm_memio_t op) {
    arm_writeh_n(op.addr, arm_memio_reg_get(op.rt));

    arm_cycles_s_to_n();
}

static void arm_memio_strd(arm_memio_t op) {
    arm_write_n(op.addr + 0, arm_memio_reg_get(op.rt));
    arm_write_s(op.addr + 4, arm_memio_reg_get(op.rt2));

    arm_cycles_s_to_n();
}

static void arm_memio_load_usr(arm_memio_t op, arm_size_e size) {
    int8_t mode = arm_r.cpsr & 0x1f;

    arm_mode_set(ARM_USR);

    switch (size) {
        case BYTE:  arm_memio_ldrb(op); break;
        case HWORD: arm_memio_ldrh(op); break;
        case WORD:  arm_memio_ldr(op);  break;
        case DWORD: arm_memio_ldrd(op); break;
    }

    arm_mode_set(mode);
}

static void arm_memio_store_usr(arm_memio_t op, arm_size_e size) {
    int8_t mode = arm_r.cpsr & 0x1f;

    arm_mode_set(ARM_USR);

    switch (size) {
        case BYTE:  arm_memio_strb(op); break;
        case HWORD: arm_memio_strh(op); break;
        case WORD:  arm_memio_str(op);  break;
        case DWORD: arm_memio_strd(op); break;
    }

    arm_mode_set(mode);
}

static void arm_memio_ldrbt(arm_memio_t op) {
    arm_memio_load_usr(op, BYTE);
}

static void arm_memio_strbt(arm_memio_t op) {
    arm_memio_store_usr(op, BYTE);
}

static arm_memio_t arm_memio_mult_op() {
    uint16_t regs = (arm_op >>  0) & 0xffff;
    uint8_t  rn   = (arm_op >> 16) & 0xf;
    bool     w    = (arm_op >> 21) & 0x1;
    bool     u    = (arm_op >> 23) & 0x1;
    bool     p    = (arm_op >> 24) & 0x1;

    uint8_t i, cnt = 0;

    for (i = 0; i < 16; i++) {
        if (regs & (1 << i)) cnt += 4;
    }

    arm_memio_t op = {
        .regs = regs,
        .rn   = rn,
        .addr = arm_r.r[rn] & ~3
    };

    if (!u)     op.addr -= cnt;
    if (u == p) op.addr += 4;

    if (w) {
        if (u)
            op.disp =  cnt;
        else
            op.disp = -cnt;
    } else {
        op.disp = 0;
    }

    return op;
}

static arm_memio_t arm_memio_imm_op() {
    uint16_t imm = (arm_op >>  0) & 0xfff;
    uint8_t  rt  = (arm_op >> 12) & 0xf;
    uint8_t  rn  = (arm_op >> 16) & 0xf;
    bool     w   = (arm_op >> 21) & 0x1;
    bool     u   = (arm_op >> 23) & 0x1;
    bool     p   = (arm_op >> 24) & 0x1;

    arm_memio_t op = {
        .rt   = rt,
        .addr = arm_r.r[rn]
    };

    if (rn == 15) op.addr &= ~3;

    int32_t disp;

    if (u)
        disp =  imm;
    else
        disp = -imm;

    if (p)       op.addr     += disp;
    if (!p || w) arm_r.r[rn] += disp;

    return op;
}

static arm_memio_t arm_memio_reg_op() {
    uint8_t rm   = (arm_op >>  0) & 0xf;
    uint8_t type = (arm_op >>  5) & 0x3;
    uint8_t imm  = (arm_op >>  7) & 0x1f;
    uint8_t rt   = (arm_op >> 12) & 0xf;
    uint8_t rn   = (arm_op >> 16) & 0xf;
    bool    w    = (arm_op >> 21) & 0x1;
    bool    u    = (arm_op >> 23) & 0x1;
    bool    p    = (arm_op >> 24) & 0x1;

    arm_memio_t op = {
        .rt   = rt,
        .addr = arm_r.r[rn]
    };

    if (rn == 15) op.addr &= ~3;

    arm_shifter_t shift = arm_data_regi(rm, type, imm);

    int32_t disp;

    if (u)
        disp =  shift.val;
    else
        disp = -shift.val;

    if (p)       op.addr     += disp;
    if (!p || w) arm_r.r[rn] += disp;

    return op;
}

static arm_memio_t arm_memio_immt_op() {
    uint32_t imm = (arm_op >>  0) & 0xfff;
    uint8_t  rt  = (arm_op >> 12) & 0xf;
    uint8_t  rn  = (arm_op >> 16) & 0xf;
    bool     u   = (arm_op >> 23) & 0x1;

    arm_memio_t op = {
        .rt   = rt,
        .addr = arm_r.r[rn]
    };

    if (u)
        arm_r.r[rn] += imm;
    else
        arm_r.r[rn] -= imm;

    return op;
}

static arm_memio_t arm_memio_regt_op() {
    uint8_t rm   = (arm_op >>  0) & 0xf;
    uint8_t type = (arm_op >>  5) & 0x3;
    uint8_t imm  = (arm_op >>  7) & 0x1f;
    uint8_t rt   = (arm_op >> 12) & 0xf;
    uint8_t rn   = (arm_op >> 16) & 0xf;
    bool    u    = (arm_op >> 23) & 0x1;

    arm_memio_t op = {
        .rt   = rt,
        .addr = arm_r.r[rn]
    };

    arm_shifter_t shift = arm_data_regi(rm, type, imm);

    if (u)
        arm_r.r[rn] += shift.val;
    else
        arm_r.r[rn] -= shift.val;

    return op;
}

static arm_memio_t arm_memio_immdh_op() {
    uint8_t  rt  = (arm_op >> 12) & 0xf;
    uint8_t  rn  = (arm_op >> 16) & 0xf;
    bool     w   = (arm_op >> 21) & 0x1;
    bool     u   = (arm_op >> 23) & 0x1;
    bool     p   = (arm_op >> 24) & 0x1;

    arm_memio_t op = {
        .rt   = rt,
        .rt2  = rt | 1,
        .addr = arm_r.r[rn]
    };

    if (rn == 15) op.addr &= ~3;

    uint16_t imm;
    int32_t disp;

    imm  = (arm_op >> 0) & 0xf;
    imm |= (arm_op >> 4) & 0xf0;

    if (u)
        disp =  imm;
    else
        disp = -imm;

    if (p)       op.addr     += disp;
    if (!p || w) arm_r.r[rn] += disp;

    return op;
}

static arm_memio_t arm_memio_regdh_op() {
    uint8_t rm = (arm_op >>  0) & 0xf;
    uint8_t rt = (arm_op >> 12) & 0xf;
    uint8_t rn = (arm_op >> 16) & 0xf;
    bool    w  = (arm_op >> 21) & 0x1;
    bool    u  = (arm_op >> 23) & 0x1;
    bool    p  = (arm_op >> 24) & 0x1;

    arm_memio_t op = {
        .rt   = rt,
        .rt2  = rt | 1,
        .addr = arm_r.r[rn]
    };

    if (rn == 15) op.addr &= ~3;

    int32_t disp;

    if (u)
        disp =  arm_r.r[rm];
    else
        disp = -arm_r.r[rm];

    if (p)       op.addr     += disp;
    if (!p || w) arm_r.r[rn] += disp;

    return op;
}

static arm_memio_t t16_memio_mult_op() {
    uint8_t regs = (arm_op >> 0) & 0xff;
    uint8_t rn   = (arm_op >> 8) & 0x7;

    uint8_t i, cnt = 0;

    for (i = 0; i < 8; i++) {
        if (regs & (1 << i)) cnt += 4;
    }

    arm_memio_t op = {
        .regs = regs,
        .rn   = rn,
        .addr = arm_r.r[rn] & ~3,
        .disp = cnt
    };

    return op;
}

static inline arm_memio_t t16_memio_imm5_op(int8_t step) {
    uint8_t rt  = (arm_op >> 0) & 0x7;
    uint8_t imm = (arm_op >> 6) & 0x1f;
    uint8_t rn  = (arm_op >> 3) & 0x7;

    uint32_t addr = arm_r.r[rn] + imm * step;

    arm_memio_t op = {
        .rt   = rt,
        .addr = addr
    };

    return op;
}

static inline arm_memio_t t16_memio_imm8sp_op() {
    uint16_t imm = (arm_op << 2) & 0x3fc;
    uint8_t  rt  = (arm_op >> 8) & 0x7;

    uint32_t addr = arm_r.r[13] + imm;

    arm_memio_t op = {
        .rt   = rt,
        .addr = addr
    };

    return op;
}

static inline arm_memio_t t16_memio_imm8pc_op() {
    uint16_t imm = (arm_op << 2) & 0x3fc;
    uint8_t  rt  = (arm_op >> 8) & 0x7;

    uint32_t addr = (arm_r.r[15] & ~3) + imm;

    arm_memio_t op = {
        .rt   = rt,
        .addr = addr
    };

    return op;
}

static arm_memio_t t16_memio_reg_op() {
    uint8_t rt = (arm_op >> 0) & 0x7;
    uint8_t rn = (arm_op >> 3) & 0x7;
    uint8_t rm = (arm_op >> 6) & 0x7;

    uint32_t addr = arm_r.r[rn] + arm_r.r[rm];

    arm_memio_t op = {
        .rt   = rt,
        .addr = addr
    };

    return op;
}

//Parallel arithmetic
typedef struct {
    arm_word lhs;
    arm_word rhs;
    uint8_t  rd;
} arm_parith_t;

static void arm_parith_qadd(arm_parith_t op) {
    int64_t lhs = op.lhs.w;
    int64_t rhs = op.rhs.w;

    arm_r.r[op.rd] = arm_ssatq(lhs + rhs);
}

static void arm_parith_qsub(arm_parith_t op) {
    int64_t lhs = op.lhs.w;
    int64_t rhs = op.rhs.w;

    arm_r.r[op.rd] = arm_ssatq(lhs - rhs);
}

static void arm_parith_qdadd(arm_parith_t op) {
    int64_t lhs = op.lhs.w;
    int64_t rhs = op.rhs.w;

    int32_t doubled = arm_ssatq(rhs << 1);

    arm_r.r[op.rd] = arm_ssatq(lhs + doubled);
}

static void arm_parith_qdsub(arm_parith_t op) {
    int64_t lhs = op.lhs.w;
    int64_t rhs = op.rhs.w;

    int32_t doubled = arm_ssatq(rhs << 1);

    arm_r.r[op.rd] = arm_ssatq(lhs - doubled);
}

static arm_parith_t arm_parith_op() {
    uint8_t rm = (arm_op >>  0) & 0xf;
    uint8_t rd = (arm_op >> 12) & 0xf;
    uint8_t rn = (arm_op >> 16) & 0xf;

    arm_parith_t op = {
        .lhs.w = arm_r.r[rm],
        .rhs.w = arm_r.r[rn],
        .rd    = rd
    };

    return op;
}

//Add with Carry
static void arm_adc_imm() {
    arm_arith_add(arm_data_imm_op(), ARM_ARITH_CARRY);
}

static void arm_adc_regi() {
    arm_arith_add(arm_data_regi_op(), ARM_ARITH_CARRY);
}

static void arm_adc_regr() {
    arm_arith_add(arm_data_regr_op(), ARM_ARITH_CARRY);
}

static void t16_adc_rdn3() {
    arm_arith_add(t16_data_rdn3_op(), ARM_ARITH_CARRY);
}

//Add
static void arm_add_imm() {
    arm_arith_add(arm_data_imm_op(), ARM_ARITH_NO_C);
}

static void arm_add_regi() {
    arm_arith_add(arm_data_regi_op(), ARM_ARITH_NO_C);
}

static void arm_add_regr() {
    arm_arith_add(arm_data_regr_op(), ARM_ARITH_NO_C);
}

static void t16_add_imm3() {
    arm_arith_add(t16_data_imm3_op(), ARM_ARITH_NO_C);
}

static inline void t16_add_imm8() {
    arm_arith_add(t16_data_imm8_op(), ARM_ARITH_NO_C);
}

static void t16_add_reg() {
    arm_arith_add(t16_data_reg_op(), ARM_ARITH_NO_C);
}

static void t16_add_rdn4() {
    arm_arith_add(t16_data_rdn4_op(ARM_FLAG_KEEP), ARM_ARITH_NO_C);
}

static void t16_add_sp7() {
    arm_arith_add(t16_data_imm7sp_op(), ARM_ARITH_NO_C);
}

static void t16_add_sp8() {
    arm_arith_add(t16_data_imm8sp_op(), ARM_ARITH_NO_C);
}

static void t16_adr() {
    arm_arith_add(t16_data_imm8pc_op(), ARM_ARITH_NO_C);
}

//And
static void arm_and_imm() {
    arm_logic(arm_data_imm_op(), AND);
}

static void arm_and_regi() {
    arm_logic(arm_data_regi_op(), AND);
}

static void arm_and_regr() {
    arm_logic(arm_data_regr_op(), AND);
}

static void t16_and_rdn3() {
    arm_logic(t16_data_rdn3_op(), AND);
}

//Bit Shift (ASR, LSL, ...)
static void arm_shift_imm() {
    arm_logic(arm_data_regi_op(), SHIFT);
}

static void arm_shift_reg() {
    arm_logic(arm_data_regr_op(), SHIFT);
}

//Arithmetic Shift Right
static inline void t16_asr_imm5() {
    arm_asr(t16_data_imm5_op(ARM_SHIFT_RIGHT));
}

static void t16_asr_rdn3() {
    arm_asr(t16_data_rdn3_op());
}

//Branch
static void arm_b() {
    int32_t imm = arm_op;

    imm <<= 8;
    imm >>= 6;

    arm_r.r[15] += imm;

    arm_load_pipe();

    // Idle-loop short-circuit: an ARM `b .` (target = current PC, i.e. a
    // tight spin) is the cart waiting for an IRQ to wake it — semantically
    // equivalent to SWI 0x02 (HALT). Set int_halt so the inner arm_exec
    // loop ends this scanline; trigger_irq() in io/timer/dma will clear
    // int_halt and dispatch the IRQ when one fires, exactly the same path
    // SWI HALT uses. r[15] is left at the branch's natural post-execute
    // value (== branch_pc + 8 in ARM pipeline), so saved_lr in
    // hle_irq_enter resolves to branch_pc on return — control resumes on
    // the branch and either re-spins until the next IRQ or falls through
    // because the IRQ handler's side effect cleared the wait condition.
    if (imm == -8) int_halt = true;
}

static inline void t16_b_imm11() {
    int32_t imm = arm_op;

    imm <<= 21;
    imm >>= 20;

    arm_r.r[15] += imm;

    arm_load_pipe();

    // Same idle-loop short-circuit as arm_b, for the Thumb encoding of
    // `b .` (0xE7FE, imm == -4). See arm_b's comment for the rationale.
    if (imm == -4) int_halt = true;
}

//Thumb Conditional Branches
static void t16_b_imm8() {
    int32_t imm = arm_op;

    imm <<= 24;
    imm >>= 23;

    int8_t cond = (arm_op >> 8) & 0xf;

    if (arm_cond(cond)) {
        arm_r.r[15] += imm;

        arm_load_pipe();
    }
}

//Bit Clear
static void arm_bic_imm() {
    arm_logic(arm_data_imm_op(), BIC);
}

static void arm_bic_regi() {
    arm_logic(arm_data_regi_op(), BIC);
}

static void arm_bic_regr() {
    arm_logic(arm_data_regr_op(), BIC);
}

static void t16_bic_rdn3() {
    arm_logic(t16_data_rdn3_op(), BIC);
}

//Breakpoint
void arm_bkpt() {
    arm_int(ARM_VEC_PABT, ARM_ABT);
}

void t16_bkpt() {
    arm_int(ARM_VEC_PABT, ARM_ABT);
}

//Branch with Link
static void arm_bl() {
    int32_t imm = arm_op;

    imm <<= 8;
    imm >>= 6;

    arm_r.r[14] =  arm_r.r[15] - ARM_WORD_SZ;
    arm_r.r[15] = (arm_r.r[15] & ~3) + imm;

    arm_load_pipe();
}

//Branch with Link and Exchange
static void arm_blx_imm() {
    int32_t imm = arm_op;

    imm <<= 8;
    imm >>= 6;

    imm |= (arm_op >> 23) & 2;

    arm_flag_set(ARM_T, true);

    arm_r.r[14]  = arm_r.r[15] - ARM_WORD_SZ;
    arm_r.r[15] += imm;

    arm_load_pipe();
}

static void arm_blx_reg() {
    uint8_t rm = arm_op & 0xf;

    arm_r.r[14] = arm_r.r[15] - ARM_WORD_SZ;
    arm_r.r[15] = arm_r.r[rm];

    arm_interwork();
}

static void t16_blx() {
    uint8_t rm = (arm_op >> 3) & 0xf;

    arm_r.r[14] = (arm_r.r[15] - ARM_HWORD_SZ) | 1;
    arm_r.r[15] =  arm_r.r[rm];

    arm_interwork();
}

static void t16_blx_h2() {
    int32_t imm = arm_op;

    imm <<= 21;
    imm >>= 9;

    arm_r.r[14]  = arm_r.r[15];
    arm_r.r[14] += imm;
}

static void t16_blx_lrimm() {
    uint32_t imm = arm_r.r[14];

    imm += (arm_op & 0x7ff) << 1;

    arm_r.r[14] = (arm_r.r[15] - ARM_HWORD_SZ) | 1;
    arm_r.r[15] = imm & ~1;
}

static void t16_blx_h1() {
    t16_blx_lrimm();
    arm_interwork();
}

static void t16_blx_h3() {
    t16_blx_lrimm();
    arm_load_pipe();
}

//Branch and Exchange
static void arm_bx() {
    uint8_t rm = arm_op & 0xf;

    arm_r.r[15] = arm_r.r[rm];

    arm_interwork();
}

static void t16_bx() {
    uint8_t rm = (arm_op >> 3) & 0xf;

    arm_r.r[15] = arm_r.r[rm];

    arm_interwork();
}

//Coprocessor Data Processing
static void arm_cdp() {
    //No coprocessors used
}

static void arm_cdp2() {
    //No coprocessors used
}

//Count Leading Zeros
static void arm_clz() {
    arm_count_zeros((arm_op >> 12) & 0xf);
}

//Compare Negative
static void arm_cmn_imm() {
    arm_arith_cmn(arm_data_imm_op());
}

static void arm_cmn_regi() {
    arm_arith_cmn(arm_data_regi_op());
}

static void arm_cmn_regr() {
    arm_arith_cmn(arm_data_regr_op());
}

static void t16_cmn_rdn3() {
    arm_arith_cmn(t16_data_rdn3_op());
}

//Compare
static void arm_cmp_imm() {
    arm_arith_cmp(arm_data_imm_op());
}

static void arm_cmp_regi() {
    arm_arith_cmp(arm_data_regi_op());
}

static void arm_cmp_regr() {
    arm_arith_cmp(arm_data_regr_op());
}

static inline void t16_cmp_imm8() {
    arm_arith_cmp(t16_data_imm8_op());
}

static void t16_cmp_rdn3() {
    arm_arith_cmp(t16_data_rdn3_op());
}

static void t16_cmp_rdn4() {
    arm_arith_cmp(t16_data_rdn4_op(true));
}

//Exclusive Or
static void arm_eor_imm() {
    arm_logic(arm_data_imm_op(), EOR);
}

static void arm_eor_regi() {
    arm_logic(arm_data_regi_op(), EOR);
}

static void arm_eor_regr() {
    arm_logic(arm_data_regr_op(), EOR);
}

static void t16_eor_rdn3() {
    arm_logic(t16_data_rdn3_op(), EOR);
}

//Load Coprocessor
static void arm_ldc() {
    //No coprocessors used
}

static void arm_ldc2() {
    //No coprocessors used
}

//Load Multiple
static void arm_ldm() {
    arm_memio_ldm(arm_memio_mult_op());
}

static void arm_ldm_usr() {
    arm_memio_ldm_usr(arm_memio_mult_op());
}

static void t16_ldm() {
    arm_memio_ldm(t16_memio_mult_op());
}

//Load Register
static void arm_ldr_imm() {
    arm_memio_ldr(arm_memio_imm_op());
}

static void arm_ldr_reg() {
    arm_memio_ldr(arm_memio_reg_op());
}

static inline void t16_ldr_imm5() {
    arm_memio_ldr(t16_memio_imm5_op(ARM_WORD_SZ));
}

static inline void t16_ldr_sp8() {
    arm_memio_ldr(t16_memio_imm8sp_op());
}

static inline void t16_ldr_pc8() {
    arm_memio_ldr(t16_memio_imm8pc_op());
}

static void t16_ldr_reg() {
    arm_memio_ldr(t16_memio_reg_op());
}

//Load Register Byte
static void arm_ldrb_imm() {
    arm_memio_ldrb(arm_memio_imm_op());
}

static void arm_ldrb_reg() {
    arm_memio_ldrb(arm_memio_reg_op());
}

static void t16_ldrb_imm5() {
    arm_memio_ldrb(t16_memio_imm5_op(ARM_BYTE_SZ));
}

static void t16_ldrb_reg() {
    arm_memio_ldrb(t16_memio_reg_op());
}

//Load Register Byte Unprivileged
static void arm_ldrbt_imm() {
    arm_memio_ldrbt(arm_memio_immt_op());
}

static void arm_ldrbt_reg() {
    arm_memio_ldrbt(arm_memio_regt_op());
}

//Load Register Dual
static void arm_ldrd_imm() {
    arm_memio_ldrd(arm_memio_immdh_op());
}

static void arm_ldrd_reg() {
    arm_memio_ldrd(arm_memio_regdh_op());
}

//Load Register Halfword
static void arm_ldrh_imm() {
    arm_memio_ldrh(arm_memio_immdh_op());
}

static void arm_ldrh_reg() {
    arm_memio_ldrh(arm_memio_regdh_op());
}

static void t16_ldrh_imm5() {
    arm_memio_ldrh(t16_memio_imm5_op(ARM_HWORD_SZ));
}

static void t16_ldrh_reg() {
    arm_memio_ldrh(t16_memio_reg_op());
}

//Load Register Signed Byte
static void arm_ldrsb_imm() {
    arm_memio_ldrsb(arm_memio_immdh_op());
}

static void arm_ldrsb_reg() {
    arm_memio_ldrsb(arm_memio_regdh_op());
}

static void t16_ldrsb_reg() {
    arm_memio_ldrsb(t16_memio_reg_op());
}

//Load Register Signed Halfword
static void arm_ldrsh_imm() {
    arm_memio_ldrsh(arm_memio_immdh_op());
}

static void arm_ldrsh_reg() {
    arm_memio_ldrsh(arm_memio_regdh_op());
}

static void t16_ldrsh_reg() {
    arm_memio_ldrsh(t16_memio_reg_op());
}

//Logical Shift Left
static inline void t16_lsl_imm5() {
    arm_lsl(t16_data_imm5_op(ARM_SHIFT_LEFT));
}

static void t16_lsl_rdn3() {
    arm_lsl(t16_data_rdn3_op());
}

//Logical Shift Right
static inline void t16_lsr_imm5() {
    arm_lsr(t16_data_imm5_op(ARM_SHIFT_RIGHT));
}

static void t16_lsr_rdn3() {
    arm_lsr(t16_data_rdn3_op());
}

//Move to Coprocessor from Register
static void arm_mcr() {
    //No coprocessors used
}

static void arm_mcr2() {
    //No coprocessors used
}

//Move to Coprocessor from Two Registers
static void arm_mcrr() {
    //No coprocessors used
}

static void arm_mcrr2() {
    //No coprocessors used
}

//Multiply Add
static void arm_mla() {
    arm_mpy_add(arm_mpy_op());
}

//Move
static void arm_mov_imm12() {
    arm_data_t op = arm_data_imm_op();

    arm_r.r[op.rd] = op.rhs;

    arm_logic_set(op, op.rhs);
}

static inline void t16_mov_imm() {
    uint8_t imm = (arm_op >> 0) & 0xff;
    uint8_t rd  = (arm_op >> 8) & 0x7;

    arm_r.r[rd] = imm;

    arm_setn(arm_r.r[rd]);
    arm_setz(arm_r.r[rd]);
}

static void t16_mov_rd4() {
    uint8_t rm = (arm_op >> 3) & 0xf;

    uint8_t rd;

    rd  = (arm_op >> 0) & 7;
    rd |= (arm_op >> 4) & 8;

    arm_r.r[rd] = arm_r.r[rm];

    if (rd == 15) {
        arm_r.r[rd] &= ~1;

        arm_load_pipe();
    }
}

static void t16_mov_rd3() {
    int8_t rd = (arm_op >> 0) & 7;
    int8_t rm = (arm_op >> 3) & 7;

    arm_r.r[rd] = arm_r.r[rm];

    arm_setn(arm_r.r[rd]);
    arm_setz(arm_r.r[rd]);
}

//Move to Register from Coprocessor
static void arm_mrc() {
    //No coprocessors used
}

static void arm_mrc2() {
    //No coprocessors used
}

//Move to Two Registers from Coprocessor
static void arm_mrrc() {
    //No coprocessors used
}

static void arm_mrrc2() {
    //No coprocessors used
}

//Move to Register from Status
static void arm_mrs() {
    arm_psr_to_reg(arm_mrs_op());
}

//Move to Status from Register (also NOP/SEV)
static void arm_msr_imm() {
    arm_reg_to_psr(arm_msr_imm_op());
}

static void arm_msr_reg() {
    arm_reg_to_psr(arm_msr_reg_op());
}

//Multiply
static void arm_mul() {
    arm_mpy(arm_mpy_op());
}

static void t16_mul() {
    arm_mpy(t16_mpy_op());
}

//Move Not
static void arm_mvn_imm() {
    arm_logic(arm_data_imm_op(), MVN);
}

static void arm_mvn_regi() {
    arm_logic(arm_data_regi_op(), MVN);
}

static void arm_mvn_regr() {
    arm_logic(arm_data_regr_op(), MVN);
}

static void t16_mvn_rdn3() {
    arm_logic(t16_data_rdn3_op(), MVN);
}

//Or
static void arm_orr_imm() {
    arm_logic(arm_data_imm_op(), ORR);
}

static void arm_orr_regi() {
    arm_logic(arm_data_regi_op(), ORR);
}

static void arm_orr_regr() {
    arm_logic(arm_data_regr_op(), ORR);
}

static void t16_orr_rdn3() {
    arm_logic(t16_data_rdn3_op(), ORR);
}

//Preload Data
static void arm_pld_imm() {
    //Performance oriented instruction, just ignore
}

static void arm_pld_reg() {
    //Performance oriented instruction, just ignore
}

//Pop
static void t16_pop() {
    uint16_t regs;

    regs  = (arm_op >> 0) & 0x00ff;
    regs |= (arm_op << 7) & 0x8000;

    uint8_t i;

    arm_r.r[13] &= ~3;

    for (i = 0; i < 16; i++) {
        if (regs & (1 << i)) {
            arm_r.r[i] = arm_read_s(arm_r.r[13]);

            arm_r.r[13] += 4;
        }
    }

    if (regs & 0x8000) {
        arm_r.r[15] &= ~1;

        arm_load_pipe();
    }
}

//Push
static void t16_push() {
    uint16_t regs;

    regs  = (arm_op >> 0) & 0x00ff;
    regs |= (arm_op << 6) & 0x4000;

    uint32_t addr = arm_r.r[13] & ~3;

    uint8_t i;

    for (i = 0; i < 16; i++) {
        if (regs & (1 << i)) addr -= 4;
    }

    arm_r.r[13] = addr;

    for (i = 0; i < 16; i++) {
        if (regs & (1 << i)) {
            arm_write_s(addr, arm_memio_reg_get(i));

            addr += 4;
        }
    }
}

//Saturating Add
static void arm_qadd() {
    arm_parith_qadd(arm_parith_op());
}

//Saturating Double and Add
static void arm_qdadd() {
    arm_parith_qdadd(arm_parith_op());
}

//Saturating Double and Subtract
static void arm_qdsub() {
    arm_parith_qdsub(arm_parith_op());
}

//Saturating Subtract
static void arm_qsub() {
    arm_parith_qsub(arm_parith_op());
}

//Rotate Right
static void t16_ror() {
    arm_ror(t16_data_rdn3_op());
}

//Reverse Subtract
static void arm_rsb_imm() {
    arm_arith_rsb(arm_data_imm_op());
}

static void arm_rsb_regi() {
    arm_arith_rsb(arm_data_regi_op());
}

static void arm_rsb_regr() {
    arm_arith_rsb(arm_data_regr_op());
}

static void t16_rsb_rdn3() {
    arm_arith_sub(t16_data_neg_op());
}

//Reverse Subtract with Carry
static void arm_rsc_imm() {
    arm_arith_rsc(arm_data_imm_op());
}

static void arm_rsc_regi() {
    arm_arith_rsc(arm_data_regi_op());
}

static void arm_rsc_regr() {
    arm_arith_rsc(arm_data_regr_op());
}

//Subtract with Carry
static void arm_sbc_imm() {
    arm_arith_sbc(arm_data_imm_op());
}

static void arm_sbc_regi() {
    arm_arith_sbc(arm_data_regi_op());
}

static void arm_sbc_regr() {
    arm_arith_sbc(arm_data_regr_op());
}

static void t16_sbc_rdn3() {
    arm_arith_sbc(t16_data_rdn3_op());
}

//Signed Multiply Accumulate
static void arm_smla__() {
    bool m = (arm_op >> 5) & 1;
    bool n = (arm_op >> 6) & 1;

    arm_mpy_smla__(arm_mpy_op(), m, n);
}

//Signed Multiply Accumulate Long
static void arm_smlal() {
    arm_mpy_smlal(arm_mpy_op());
}

//Signed Multiply Accumulate Long (Halfwords)
static void arm_smlal__() {
    bool m = (arm_op >> 5) & 1;
    bool n = (arm_op >> 6) & 1;

    arm_mpy_smlal__(arm_mpy_op(), m, n);
}

//Signed Multiply Accumulate Word by Halfword
static void arm_smlaw_() {
    arm_mpy_smlaw_(arm_mpy_op(), (arm_op >> 6) & 1);
}

//Signed Multiply
static void arm_smul() {
    bool m = (arm_op >> 5) & 1;
    bool n = (arm_op >> 6) & 1;

    arm_mpy_smul(arm_mpy_op(), m, n);
}

//Signed Multiply Long
static void arm_smull() {
    arm_mpy_smull(arm_mpy_op());
}

//Signed Multiply Word by Halfword
static void arm_smulw_() {
    arm_mpy_smulw_(arm_mpy_op(), (arm_op >> 6) & 1);
}

//Store Coprocessor
static void arm_stc() {
    //No coprocessors used
}

static void arm_stc2() {
    //No coprocessors used
}

//Store Multiple
static void arm_stm() {
    arm_memio_stm(arm_memio_mult_op());
}

static void arm_stm_usr() {
    arm_memio_stm_usr(arm_memio_mult_op());
}

static void t16_stm() {
    arm_memio_stm(t16_memio_mult_op());
}

//Store Register
static void arm_str_imm() {
    arm_memio_str(arm_memio_imm_op());
}

static void arm_str_reg() {
    arm_memio_str(arm_memio_reg_op());
}

static inline void t16_str_imm5() {
    arm_memio_str(t16_memio_imm5_op(ARM_WORD_SZ));
}

static inline void t16_str_sp8() {
    arm_memio_str(t16_memio_imm8sp_op());
}

static void t16_str_reg() {
    arm_memio_str(t16_memio_reg_op());
}

//Store Register Byte
static void arm_strb_imm() {
    arm_memio_strb(arm_memio_imm_op());
}

static void arm_strb_reg() {
    arm_memio_strb(arm_memio_reg_op());
}

static void t16_strb_imm5() {
    arm_memio_strb(t16_memio_imm5_op(ARM_BYTE_SZ));
}

static void t16_strb_reg() {
    arm_memio_strb(t16_memio_reg_op());
}

//Store Register Byte Unprivileged
static void arm_strbt_imm() {
    arm_memio_strbt(arm_memio_immt_op());
}

static void arm_strbt_reg() {
    arm_memio_strbt(arm_memio_regt_op());
}

//Store Register Dual
static void arm_strd_imm() {
    arm_memio_strd(arm_memio_immdh_op());
}

static void arm_strd_reg() {
    arm_memio_strd(arm_memio_regdh_op());
}

//Store Register Halfword
static void arm_strh_imm() {
    arm_memio_strh(arm_memio_immdh_op());
}

static void arm_strh_reg() {
    arm_memio_strh(arm_memio_regdh_op());
}

static void t16_strh_imm5() {
    arm_memio_strh(t16_memio_imm5_op(ARM_HWORD_SZ));
}

static void t16_strh_reg() {
    arm_memio_strh(t16_memio_reg_op());
}

//Subtract
static void arm_sub_imm() {
    arm_arith_sub(arm_data_imm_op());
}

static void arm_sub_regi() {
    arm_arith_sub(arm_data_regi_op());
}

static void arm_sub_regr() {
    arm_arith_sub(arm_data_regr_op());
}

static void t16_sub_imm3() {
    arm_arith_sub(t16_data_imm3_op());
}

static inline void t16_sub_imm8() {
    arm_arith_sub(t16_data_imm8_op());
}

static void t16_sub_reg() {
    arm_arith_sub(t16_data_reg_op());
}

static void t16_sub_sp7() {
    arm_arith_sub(t16_data_imm7sp_op());
}

//Supervisor Call
//
// HLE for the most common BIOS SWI calls. The replacement BIOS shipped with
// this build doesn't implement these correctly (the function-pointer table
// at 0x1F4 onwards has bogus data, sending control to low BIOS addresses),
// so we intercept the SWI before it reaches the BIOS dispatcher.
extern volatile uint32_t arm_swi_count[256];
extern volatile uint8_t  arm_swi_last;

static void hle_cpu_set(void) {
    uint32_t src     = arm_r.r[0];
    uint32_t dst     = arm_r.r[1];
    uint32_t lenmode = arm_r.r[2];
    uint32_t count   = lenmode & 0x1FFFFF;
    bool     word    = (lenmode >> 26) & 1;
    bool     fill    = (lenmode >> 24) & 1;

    if (word) {
        src &= ~3u; dst &= ~3u;
        uint32_t v = fill ? arm_read(src) : 0;
        for (uint32_t i = 0; i < count; i++) {
            if (!fill) { v = arm_read(src); src += 4; }
            arm_write(dst, v); dst += 4;
        }
    } else {
        src &= ~1u; dst &= ~1u;
        uint16_t v = fill ? (uint16_t)arm_readh(src) : 0;
        for (uint32_t i = 0; i < count; i++) {
            if (!fill) { v = (uint16_t)arm_readh(src); src += 2; }
            arm_writeh(dst, v); dst += 2;
        }
    }
    arm_r.r[0] = src;
    arm_r.r[1] = dst;
}

static void hle_cpu_fast_set(void) {
    uint32_t src     = arm_r.r[0] & ~3u;
    uint32_t dst     = arm_r.r[1] & ~3u;
    uint32_t lenmode = arm_r.r[2];
    uint32_t count   = ((lenmode & 0x1FFFFF) + 7) & ~7u;
    bool     fill    = (lenmode >> 24) & 1;

    uint32_t v = fill ? arm_read(src) : 0;
    for (uint32_t i = 0; i < count; i++) {
        if (!fill) { v = arm_read(src); src += 4; }
        arm_write(dst, v); dst += 4;
    }
    arm_r.r[0] = src;
    arm_r.r[1] = dst;
}

static void hle_div(void) {
    int32_t num = (int32_t)arm_r.r[0];
    int32_t den = (int32_t)arm_r.r[1];
    if (den != 0) {
        int32_t q = num / den;
        int32_t r = num % den;
        arm_r.r[0] = (uint32_t)q;
        arm_r.r[1] = (uint32_t)r;
        arm_r.r[3] = (uint32_t)(q < 0 ? -q : q);
    }
}

static void hle_register_ram_reset(void) {
    uint32_t flags = arm_r.r[0];
    if (flags & 0x01) memset(wram_board,  0, ewram_size);
    if (flags & 0x02) memset(wram_chip,   0, sizeof(wram_chip) - 0x200);
    if (flags & 0x04) memset(palette_ram, 0, sizeof(palette_ram));
    if (flags & 0x08) memset(vram,        0, sizeof(vram));
    if (flags & 0x10) memset(oam,         0, sizeof(oam));
}

// LZ77 decompression. The replacement BIOS we ship with often has a
// broken or missing implementation, which causes both slowdown (it runs
// as emulated ARM code in BIOS) and corruption (output is garbage).
// HLE'ing it bypasses the BIOS entirely.
//
// Format (per GBATEK):
//   r0 = source pointer (must be 4-byte aligned for VRAM variant)
//   r1 = destination pointer
//   word at *r0 = (compression_type << 0) | (decompressed_size << 8)
//                where compression_type = 0x10 (LZ77 marker)
//   following bytes = compressed stream
//
// Compressed stream format:
//   - 1 flags byte; each bit (MSB first) indicates the next item type
//   - flag bit 0: literal byte follows
//   - flag bit 1: back-reference (2 bytes follow)
//       byte 1 high nibble = (length - 3), low nibble = displacement high
//       byte 2          = displacement low (12-bit total displacement)
//       length 3..18, displacement 1..4096
//   - back-reference copies (length) bytes from (dst - displacement - 1)
//
// The VRAM variant differs only in that destination writes go through
// arm_writeh in halfword pairs (VRAM doesn't accept 8-bit writes for
// OBJ region, and the BIOS spec specifies halfword writes regardless).
static void hle_lz77_uncomp(bool to_vram) {
    uint32_t src = arm_r.r[0];
    uint32_t dst = arm_r.r[1];

    uint32_t header = arm_read(src);
    src += 4;
    uint32_t size = header >> 8;

    uint32_t written = 0;
    uint16_t pair = 0;       // halfword buffer for VRAM mode
    int      pair_have = 0;  // 0 or 1 byte buffered

    while (written < size) {
        uint8_t flags = arm_readb(src++);

        for (int i = 0; i < 8 && written < size; i++) {
            uint8_t v;

            if (flags & 0x80) {
                // Back-reference: 2 bytes
                uint8_t b1 = arm_readb(src++);
                uint8_t b2 = arm_readb(src++);
                uint32_t length = (b1 >> 4) + 3;
                uint32_t disp   = (((uint32_t)b1 & 0xF) << 8) | b2;
                disp += 1;

                for (uint32_t k = 0; k < length && written < size; k++) {
                    // Read previously-decompressed byte. For VRAM mode we
                    // need to read what we wrote; the cart-side reads of
                    // VRAM go through arm_readb which handles regions
                    // correctly.
                    v = arm_readb(dst - disp);

                    if (to_vram) {
                        if (pair_have) {
                            pair |= (uint16_t)v << 8;
                            arm_writeh(dst - 1, pair);
                            pair = 0;
                            pair_have = 0;
                        } else {
                            pair = v;
                            pair_have = 1;
                        }
                    } else {
                        arm_writeb(dst, v);
                    }
                    dst++;
                    written++;
                }
            } else {
                // Literal
                v = arm_readb(src++);
                if (to_vram) {
                    if (pair_have) {
                        pair |= (uint16_t)v << 8;
                        arm_writeh(dst - 1, pair);
                        pair = 0;
                        pair_have = 0;
                    } else {
                        pair = v;
                        pair_have = 1;
                    }
                } else {
                    arm_writeb(dst, v);
                }
                dst++;
                written++;
            }

            flags <<= 1;
        }
    }

    // Flush trailing odd byte for VRAM mode
    if (to_vram && pair_have) {
        arm_writeh(dst - 1, pair);
    }
}

static void hle_lz77_uncomp_wram(void) { hle_lz77_uncomp(false); }
static void hle_lz77_uncomp_vram(void) { hle_lz77_uncomp(true);  }

static void hle_halt(void) {
    int_halt = true;
}

// VBlankIntrWait / IntrWait: emulate the relevant side effects of the
// real BIOS implementation, then halt. The run-frame loop will fire the
// cart's IRQ when it reaches v_count = 160 (or when the requested timer/
// HBlank/VCount IRQ fires), the cart's installed IRQ handler at
// *(0x03007FFC) runs and acks IF + sets BIOSIF, and execution resumes
// past the SWI on return.
//
// Real BIOS SWI 0x05 specifically:
//   1. IME = 1
//   2. IE |= VBLANK
//   3. loop: halt; if (BIOSIF & VBLANK) { BIOSIF &= ~VBLANK; return; }
//
// Real BIOS SWI 0x04 (IntrWait):
//   r0 = discardOldFlags (if 1, clear BIOSIF before checking)
//   r1 = irqMask (which IF bits to wait for)
//   1. IME = 1
//   2. IE |= r1
//   3. if (r0) BIOSIF &= ~r1
//   4. loop: halt; if (BIOSIF & r1) { BIOSIF &= ~matched; return; }
//
// Carts that call SWI 0x05 with IME/IE not pre-set (a few do, knowing
// BIOS will enable them) would otherwise halt forever -- no IRQ can
// fire to wake us. The IME/IE writes here fix that.
static void hle_vblank_intr_wait(void) {
    int_enb_m.w |= 1;          // IME = 1
    int_enb.w   |= 0x0001;     // IE bit 0 = VBlank

    // Clear BIOSIF VBlank flag at 0x03007FF8 bit 0. We can't loop-check
    // it like real BIOS does (would need recursive arm_exec), but
    // clearing it on entry mirrors the discardOldFlags=true behaviour
    // that almost all callers want.
    wram_chip[0x7FF8] &= ~0x01;

    int_halt = true;
}

static void hle_intr_wait(void) {
    int_enb_m.w |= 1;
    int_enb.w   |= (uint16_t)arm_r.r[1];   // enable requested IRQs

    if (arm_r.r[0]) {
        // discardOldFlags: clear matching BIOSIF bits.
        uint16_t mask = (uint16_t)arm_r.r[1];
        wram_chip[0x7FF8] &= ~(uint8_t)(mask & 0xFF);
        wram_chip[0x7FF9] &= ~(uint8_t)(mask >> 8);
    }

    int_halt = true;
}

// HLE'd BIOS IRQ entry / return.
//
// The standard GBA BIOS IRQ handler at 0x128 does a fixed sequence on
// every interrupt:
//
//   STMFD SP!, {R0-R3, R12, LR}   ; push to IRQ stack
//   MOV   R0, #0x04000000         ; IO base for the user handler
//   ADR   LR, post_handler        ; return path inside BIOS
//   LDR   PC, [R0, #-4]           ; jump to *(0x03007FFC) (cart handler)
//   ;-- post_handler:
//   LDMFD SP!, {R0-R3, R12, LR}
//   SUBS  PC, LR, #4              ; restore CPSR from SPSR_irq + return
//
// The interpreter ran each of those ~10 instructions per IRQ, with a
// chunk-cache miss for every BIOS-region fetch. On a workload with many
// IRQs per frame (any game using IntrWait/VBlankIntrWait, plus HBlank/
// timer IRQs), the wall-time cost was substantial.
//
// The HLE replicates the architecturally visible effects directly in C:
// pushes the same six registers to R13_irq, sets R0/LR, jumps to the
// cart handler. On return (detected by PC == HLE_IRQ_RETURN_PC, see
// arm_load_pipe), it pops and restores CPSR. No bus traffic for the
// BIOS instructions themselves, no chunk-cache pressure, no per-op
// dispatch overhead.
//
// Bypassed only when the cart hasn't installed a handler yet
// (*(0x03007FFC) == 0); arm_check_irq falls through to the original
// arm_int(VEC_IRQ, IRQ) path so the BIOS still runs in that case
// (and lets early-boot code that depends on the BIOS's null-check
// behavior work).
volatile uint32_t arm_hle_irq_count;

static void hle_irq_enter(uint32_t handler_addr) {
    // Mirror arm_int(VEC_IRQ, IRQ) for the state-save half: pack flags,
    // snapshot CPSR, switch to IRQ mode, save SPSR_irq.
    arm_flags_to_cpsr();
    uint32_t cpsr_save = arm_r.cpsr;

    // Compute the saved return-PC the same way arm_int(VEC_IRQ, IRQ) does:
    //   ARM:   LR_irq = r[15] - 4
    //   Thumb: LR_irq = r[15]
    // (See the PC-adjust block in arm_int.)
    uint32_t saved_lr = (cpsr_save & ARM_T)
        ? arm_r.r[15]
        : arm_r.r[15] - 4;

    arm_mode_set(ARM_IRQ);
    arm_spsr_set(cpsr_save);

    // STMFD SP!, {R0-R3, R12, R14}: push 6 words to R13_irq.
    // Stack grows down; on-stack order (low→high) is R0, R1, R2, R3, R12, LR.
    arm_r.r[13] -= 24;
    uint32_t sp = arm_r.r[13];
    arm_write(sp +  0, arm_r.r[0]);
    arm_write(sp +  4, arm_r.r[1]);
    arm_write(sp +  8, arm_r.r[2]);
    arm_write(sp + 12, arm_r.r[3]);
    arm_write(sp + 16, arm_r.r[12]);
    arm_write(sp + 20, saved_lr);

    // Hand the user handler the same environment the BIOS would have:
    // R0 = IO base, LR = "where to return to" (sentinel for our HLE path).
    arm_r.r[0]  = 0x04000000;
    arm_r.r[14] = HLE_IRQ_RETURN_PC;

    // Force ARM mode for the handler (BIOS handler runs in ARM regardless
    // of what mode the cart was in when interrupted) and disable IRQs.
    arm_flag_set(ARM_T, false);
    arm_flag_set(ARM_I, true);

    arm_r.r[15] = handler_addr;
    arm_hle_irq_count++;
    arm_load_pipe();
}

static void hle_irq_return(void) {
    // Mirror the BIOS post-handler:
    //   LDMFD SP!, {R0-R3, R12, LR}
    //   SUBS  PC, LR, #4
    uint32_t sp = arm_r.r[13];
    arm_r.r[0]  = arm_read(sp +  0);
    arm_r.r[1]  = arm_read(sp +  4);
    arm_r.r[2]  = arm_read(sp +  8);
    arm_r.r[3]  = arm_read(sp + 12);
    arm_r.r[12] = arm_read(sp + 16);
    arm_r.r[14] = arm_read(sp + 20);
    arm_r.r[13] = sp + 24;

    uint32_t new_pc = arm_r.r[14] - 4;

    // SUBS PC, LR, #4 with PC as destination + S flag = restore CPSR
    // from SPSR_irq, then jump. arm_spsr_to_cpsr handles the mode
    // switch + bank-register restore + lazy-NZCV resync.
    arm_spsr_to_cpsr();
    arm_r.r[15] = new_pc;
    arm_r15_align();
}

static bool hle_swi(uint8_t swi_num) {
    arm_swi_last = swi_num;
    arm_swi_count[swi_num]++;
    switch (swi_num) {
        case 0x01: hle_register_ram_reset(); return true;
        case 0x02: hle_halt();               return true;
        case 0x04: hle_intr_wait();          return true;
        case 0x05: hle_vblank_intr_wait();   return true;
        case 0x06: hle_div();                return true;
        case 0x0B: hle_cpu_set();            return true;
        case 0x0C: hle_cpu_fast_set();       return true;
        case 0x11: hle_lz77_uncomp_wram();   return true;
        case 0x12: hle_lz77_uncomp_vram();   return true;
        default: return false;
    }
}

static void arm_svc() {
    // GBA convention: ARM SWI number is in bits 23-16 of the instruction.
    uint8_t swi_num = (arm_op >> 16) & 0xFF;
    if (hle_swi(swi_num)) return;
    arm_int(ARM_VEC_SVC, ARM_SVC);
}

static void t16_svc() {
    // Thumb SWI number is in the low 8 bits.
    uint8_t swi_num = arm_op & 0xFF;
    if (hle_swi(swi_num)) return;
    arm_int(ARM_VEC_SVC, ARM_SVC);
}

//Swap
static void arm_swp() {
    uint8_t rt2 = (arm_op >>  0) & 0xf;
    uint8_t rt  = (arm_op >> 12) & 0xf;
    uint8_t rn  = (arm_op >> 16) & 0xf;
    bool    b   = (arm_op >> 22) & 0x1;

    uint32_t val;

    if (b)
        val = arm_readb_n(arm_r.r[rn]);
    else
        val = arm_read_n(arm_r.r[rn]);

    if (b)
        arm_writeb_n(arm_r.r[rn], arm_r.r[rt2]);
    else
        arm_write_n(arm_r.r[rn], arm_r.r[rt2]);

    arm_r.r[rt] = val;

    arm_cycles++;
}

//Test Equivalence
static void arm_teq_imm() {
    arm_logic_teq(arm_data_imm_op());
}

static void arm_teq_regi() {
    arm_logic_teq(arm_data_regi_op());
}

static void arm_teq_regr() {
    arm_logic_teq(arm_data_regr_op());
}

//Test
static void arm_tst_imm() {
    arm_logic_tst(arm_data_imm_op());
}

static void arm_tst_regi() {
    arm_logic_tst(arm_data_regi_op());
}

static void arm_tst_regr() {
    arm_logic_tst(arm_data_regr_op());
}

static void t16_tst_rdn3() {
    arm_logic_tst(t16_data_rdn3_op());
}

//Unsigned Multiply Accumulate Long
static void arm_umlal() {
    arm_mpy_umlal(arm_mpy_op());
}

//Unsigned Multiply Long
static void arm_umull() {
    arm_mpy_umull(arm_mpy_op());
}

//Undefined
extern volatile uint32_t arm_undef_count;
extern volatile uint32_t arm_undef_first_pc;
extern volatile uint32_t arm_undef_first_op;
static void arm_und() {
    if (arm_undef_count == 0) {
        arm_undef_first_pc = arm_r.r[15];
        arm_undef_first_op = arm_op;
    }
    arm_undef_count++;
    arm_int(ARM_VEC_UND, ARM_UND);
}

/*
 * Decode
 */

static void arm_proc_fill(void (**arr)(), void (*proc)(), int32_t size) {
    int32_t i;

    for (i = 0; i < size; i++) {
        arr[i] = proc;
    }
}

static void arm_proc_set(void (**arr)(), void (*proc)(), uint32_t op, uint32_t mask, int32_t bits) {
    //Bits you need on op needs to be set 1 on mask
    int32_t i, j;
    int32_t zbits = 0;
    int32_t zpos[bits];

    for (i = 0; i < bits; i++) {
        if ((mask & (1 << i)) == 0) zpos[zbits++] = i;
    }

    for (i = 0; i < (1 << zbits); i++) {
        op &= mask;

        for (j = 0; j < zbits; j++) {
            op |= ((i >> j) & 1) << zpos[j];
        }

        arr[op] = proc;
    }
}

// ARM-mode dispatch tables — compressed.
//
// The original layout was `void (*arm_proc[2][4096])();` = 32 KB of
// function pointers in main RAM. Out of 4096 possible 12-bit decode keys
// per cond bank, only ~120 unique handlers exist; the rest are arm_und.
// At 32 KB the table didn't fit in the SH4A's 16 KB OC, so steady-state
// dispatch missed cache constantly.
//
// Compressed: a small handler-pointer array (1 byte index per slot) plus
// two 4096-byte index tables. Total main-RAM footprint drops from 32 KB
// to 8.5 KB, which fits comfortably in OC even with cart code/data
// competing for cache lines. Per-dispatch cost goes from one 4-byte
// pointer load (often a miss) to one 1-byte index load (hits OC) plus
// one 4-byte handler-pointer load (always hot — only 480 bytes total
// and accessed every dispatch, so guaranteed OC-resident).
//
// Index 0 is reserved for arm_und so the zero-initialized fill is
// already correct.
#define ARM_PROC_HANDLERS_MAX 128
static void (*arm_proc_handlers[ARM_PROC_HANDLERS_MAX])();
static uint8_t arm_proc_handlers_count;

uint8_t arm_proc_idx_0[4096];   // typical conds (0..14)
uint8_t arm_proc_idx_1[4096];   // cond=15 (UNCOND)

static uint8_t arm_proc_register(void (*fn)()) {
    // Linear dedupe — only ~120 unique handlers, called at init only.
    for (int i = 0; i < arm_proc_handlers_count; i++) {
        if (arm_proc_handlers[i] == fn) return (uint8_t)i;
    }
    uint8_t idx = arm_proc_handlers_count++;
    arm_proc_handlers[idx] = fn;
    return idx;
}

static void arm_proc_fill_c(uint8_t *arr, void (*proc)(), int32_t size) {
    uint8_t idx = arm_proc_register(proc);
    for (int32_t i = 0; i < size; i++) arr[i] = idx;
}

static void arm_proc_set_c(uint8_t *arr, void (*proc)(),
                           uint32_t op, uint32_t mask, int32_t bits) {
    uint8_t idx = arm_proc_register(proc);
    int32_t zbits = 0;
    int32_t zpos[bits];

    for (int32_t i = 0; i < bits; i++) {
        if ((mask & (1 << i)) == 0) zpos[zbits++] = i;
    }

    for (int32_t i = 0; i < (1 << zbits); i++) {
        op &= mask;

        for (int32_t j = 0; j < zbits; j++) {
            op |= ((i >> j) & 1) << zpos[j];
        }

        arr[op] = idx;
    }
}

// Thumb dispatch table in XYRAM: hit on every Thumb instruction, and Thumb
// is the dominant mode in most GBA games. 2048 entries × 4 bytes = 8 KB,
// fits comfortably in XYRAM (16 KB). Each lookup avoids a main-RAM bus
// transaction (~117 MHz) on cache miss. Not compressed (XYRAM never
// misses, so the indirection would only add a load).
void (*thumb_proc[2048])() GXRAM;

static void arm_proc_init() {
    //Format 27:20,7:4
    arm_proc_fill_c(arm_proc_idx_0, arm_und, 4096);
    arm_proc_fill_c(arm_proc_idx_1, arm_und, 4096);

    //Conditional
    arm_proc_set_c(arm_proc_idx_0, arm_adc_imm,    0b001010100000, 0b111111100000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_adc_regi,   0b000010100000, 0b111111100001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_adc_regr,   0b000010100001, 0b111111101001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_add_imm,    0b001010000000, 0b111111100000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_add_regi,   0b000010000000, 0b111111100001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_add_regr,   0b000010000001, 0b111111101001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_and_imm,    0b001000000000, 0b111111100000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_and_regi,   0b000000000000, 0b111111100001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_and_regr,   0b000000000001, 0b111111101001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_shift_imm,  0b000110100000, 0b111111100001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_shift_reg,  0b000110100001, 0b111111101001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_b,          0b101000000000, 0b111100000000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_bic_imm,    0b001111000000, 0b111111100000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_bic_regi,   0b000111000000, 0b111111100001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_bic_regr,   0b000111000001, 0b111111101001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_bkpt,       0b000100100111, 0b111111111111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_bl,         0b101100000000, 0b111100000000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_blx_reg,    0b000100100011, 0b111111111111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_bx,         0b000100100001, 0b111111111111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_cdp,        0b111000000000, 0b111100000001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_clz,        0b000101100001, 0b111111111111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_cmn_imm,    0b001101110000, 0b111111110000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_cmn_regi,   0b000101110000, 0b111111110001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_cmn_regr,   0b000101110001, 0b111111111001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_cmp_imm,    0b001101010000, 0b111111110000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_cmp_regi,   0b000101010000, 0b111111110001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_cmp_regr,   0b000101010001, 0b111111111001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_eor_imm,    0b001000100000, 0b111111100000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_eor_regi,   0b000000100000, 0b111111100001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_eor_regr,   0b000000100001, 0b111111101001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_ldc,        0b110000010000, 0b111000010000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_ldm,        0b100000010000, 0b111001010000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_ldm_usr,    0b100001010000, 0b111001010000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_ldr_imm,    0b010000010000, 0b111001010000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_ldr_reg,    0b011000010000, 0b111001010001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_ldrb_imm,   0b010001010000, 0b111001010000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_ldrb_reg,   0b011001010000, 0b111001010001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_ldrbt_imm,  0b010001110000, 0b111101110000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_ldrbt_reg,  0b011001110000, 0b111101110001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_ldrd_imm,   0b000001001101, 0b111001011111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_ldrd_reg,   0b000000001101, 0b111001011111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_ldrh_imm,   0b000001011011, 0b111001011111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_ldrh_reg,   0b000000011011, 0b111001011111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_ldrsb_imm,  0b000001011101, 0b111001011111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_ldrsb_reg,  0b000000011101, 0b111001011111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_ldrsh_imm,  0b000001011111, 0b111001011111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_ldrsh_reg,  0b000000011111, 0b111001011111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_mcr,        0b111000000001, 0b111100010001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_mcrr,       0b110001000000, 0b111111110000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_mla,        0b000000101001, 0b111111101111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_mov_imm12,  0b001110100000, 0b111111100000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_mrc,        0b111000010001, 0b111100010001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_mrrc,       0b110001010000, 0b111111110000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_mrs,        0b000100000000, 0b111110111111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_msr_imm,    0b001100100000, 0b111111110000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_msr_reg,    0b000100100000, 0b111110111111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_mul,        0b000000001001, 0b111111101111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_mvn_imm,    0b001111100000, 0b111111100000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_mvn_regi,   0b000111100000, 0b111111100001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_mvn_regr,   0b000111100001, 0b111111101001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_orr_imm,    0b001110000000, 0b111111100000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_orr_regi,   0b000110000000, 0b111111100001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_orr_regr,   0b000110000001, 0b111111101001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_qadd,       0b000100000101, 0b111111111111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_qdadd,      0b000101000101, 0b111111111111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_qdsub,      0b000101100101, 0b111111111111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_qsub,       0b000100100101, 0b111111111111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_rsb_imm,    0b001001100000, 0b111111100000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_rsb_regi,   0b000001100000, 0b111111100001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_rsb_regr,   0b000001100001, 0b111111101001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_rsc_imm,    0b001011100000, 0b111111100000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_rsc_regi,   0b000011100000, 0b111111100001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_rsc_regr,   0b000011100001, 0b111111101001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_sbc_imm,    0b001011000000, 0b111111100000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_sbc_regi,   0b000011000000, 0b111111100001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_sbc_regr,   0b000011000001, 0b111111101001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_smla__,     0b000100001000, 0b111111111001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_smlal,      0b000011101001, 0b111111101111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_smlal__,    0b000101001000, 0b111111111001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_smlaw_,     0b000100101000, 0b111111111011, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_smul,       0b000101101000, 0b111111111001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_smull,      0b000011001001, 0b111111101111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_smulw_,     0b000100101010, 0b111111111011, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_stc,        0b110000000000, 0b111000010000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_stm,        0b100000000000, 0b111001010000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_stm_usr,    0b100001000000, 0b111001010000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_str_imm,    0b010000000000, 0b111001010000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_str_reg,    0b011000000000, 0b111001010001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_strb_imm,   0b010001000000, 0b111001010000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_strb_reg,   0b011001000000, 0b111001010001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_strbt_imm,  0b010001100000, 0b111101110000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_strbt_reg,  0b011001100000, 0b111101110001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_strd_imm,   0b000001001111, 0b111001011111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_strd_reg,   0b000000001111, 0b111001011111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_strh_imm,   0b000001001011, 0b111001011111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_strh_reg,   0b000000001011, 0b111001011111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_sub_imm,    0b001001000000, 0b111111100000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_sub_regi,   0b000001000000, 0b111111100001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_sub_regr,   0b000001000001, 0b111111101001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_svc,        0b111100000000, 0b111100000000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_swp,        0b000100001001, 0b111110111111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_teq_imm,    0b001100110000, 0b111111110000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_teq_regi,   0b000100110000, 0b111111110001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_teq_regr,   0b000100110001, 0b111111111001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_tst_imm,    0b001100010000, 0b111111110000, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_tst_regi,   0b000100010000, 0b111111110001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_tst_regr,   0b000100010001, 0b111111111001, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_umlal,      0b000010101001, 0b111111101111, 12);
    arm_proc_set_c(arm_proc_idx_0, arm_umull,      0b000010001001, 0b111111101111, 12);

    //Unconditional
    arm_proc_set_c(arm_proc_idx_1, arm_blx_imm,    0b101000000000, 0b111000000000, 12);
    arm_proc_set_c(arm_proc_idx_1, arm_cdp2,       0b111000000000, 0b111100000001, 12);
    arm_proc_set_c(arm_proc_idx_1, arm_ldc2,       0b110000010000, 0b111000010000, 12);
    arm_proc_set_c(arm_proc_idx_1, arm_mcr2,       0b111000000001, 0b111100010001, 12);
    arm_proc_set_c(arm_proc_idx_1, arm_mcrr2,      0b110001000000, 0b111111110000, 12);
    arm_proc_set_c(arm_proc_idx_1, arm_mrc2,       0b111000010001, 0b111100010001, 12);
    arm_proc_set_c(arm_proc_idx_1, arm_mrrc2,      0b110001010000, 0b111111110000, 12);
    arm_proc_set_c(arm_proc_idx_1, arm_pld_imm,    0b010101010000, 0b111101110000, 12);
    arm_proc_set_c(arm_proc_idx_1, arm_pld_reg,    0b011101010000, 0b111101110000, 12);
    arm_proc_set_c(arm_proc_idx_1, arm_stc2,       0b110000000000, 0b111000010000, 12);
}

static void thumb_proc_init() {
    //Format 15:5
    arm_proc_fill(thumb_proc, arm_und, 2048);

    arm_proc_set(thumb_proc, t16_adc_rdn3,   0b01000001010, 0b11111111110, 11);
    arm_proc_set(thumb_proc, t16_add_imm3,   0b00011100000, 0b11111110000, 11);
    arm_proc_set(thumb_proc, t16_add_imm8,   0b00110000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_add_reg,    0b00011000000, 0b11111110000, 11);
    arm_proc_set(thumb_proc, t16_add_rdn4,   0b01000100000, 0b11111111000, 11);
    arm_proc_set(thumb_proc, t16_add_sp7,    0b10110000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_add_sp8,    0b10101000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_adr,        0b10100000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_and_rdn3,   0b01000000000, 0b11111111110, 11);
    arm_proc_set(thumb_proc, t16_asr_imm5,   0b00010000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_asr_rdn3,   0b01000001000, 0b11111111110, 11);
    arm_proc_set(thumb_proc, t16_b_imm8,     0b11010000000, 0b11110000000, 11);
    arm_proc_set(thumb_proc, t16_b_imm11,    0b11100000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_bic_rdn3,   0b01000011100, 0b11111111110, 11);
    arm_proc_set(thumb_proc, t16_bkpt,       0b10111110000, 0b11111111000, 11);
    // t16_blx (Thumb BLX register, 0x4780-0x479F) and t16_blx_h1 (Thumb BLX
    // immediate suffix, 0xE800-0xEFFF) are ARMv5+ instructions that do not
    // exist on the GBA's ARM7TDMI (ARMv4T). Leaving them registered here
    // caused undefined Thumb encodings to be silently treated as BLX,
    // producing wild PC jumps. Both ranges are now undefined (fall through
    // to arm_und) which is what real hardware does.
    arm_proc_set(thumb_proc, t16_blx_h2,     0b11110000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_blx_h3,     0b11111000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_bx,         0b01000111000, 0b11111111100, 11);
    arm_proc_set(thumb_proc, t16_cmn_rdn3,   0b01000010110, 0b11111111110, 11);
    arm_proc_set(thumb_proc, t16_cmp_imm8,   0b00101000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_cmp_rdn3,   0b01000010100, 0b11111111110, 11);
    arm_proc_set(thumb_proc, t16_cmp_rdn4,   0b01000101000, 0b11111111000, 11);
    arm_proc_set(thumb_proc, t16_eor_rdn3,   0b01000000010, 0b11111111110, 11);
    arm_proc_set(thumb_proc, t16_ldm,        0b11001000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_ldr_imm5,   0b01101000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_ldr_sp8,    0b10011000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_ldr_pc8,    0b01001000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_ldr_reg,    0b01011000000, 0b11111110000, 11);
    arm_proc_set(thumb_proc, t16_ldrb_imm5,  0b01111000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_ldrb_reg,   0b01011100000, 0b11111110000, 11);
    arm_proc_set(thumb_proc, t16_ldrh_imm5,  0b10001000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_ldrh_reg,   0b01011010000, 0b11111110000, 11);
    arm_proc_set(thumb_proc, t16_ldrsb_reg,  0b01010110000, 0b11111110000, 11);
    arm_proc_set(thumb_proc, t16_ldrsh_reg,  0b01011110000, 0b11111110000, 11);
    arm_proc_set(thumb_proc, t16_lsl_imm5,   0b00000000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_lsl_rdn3,   0b01000000100, 0b11111111110, 11);
    arm_proc_set(thumb_proc, t16_lsr_imm5,   0b00001000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_lsr_rdn3,   0b01000000110, 0b11111111110, 11);
    arm_proc_set(thumb_proc, t16_mov_imm,    0b00100000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_mov_rd4,    0b01000110000, 0b11111111000, 11);
    arm_proc_set(thumb_proc, t16_mov_rd3,    0b00000000000, 0b11111111110, 11);
    arm_proc_set(thumb_proc, t16_mul,        0b01000011010, 0b11111111110, 11);
    arm_proc_set(thumb_proc, t16_mvn_rdn3,   0b01000011110, 0b11111111110, 11);
    arm_proc_set(thumb_proc, t16_orr_rdn3,   0b01000011000, 0b11111111110, 11);
    arm_proc_set(thumb_proc, t16_pop,        0b10111100000, 0b11111110000, 11);
    arm_proc_set(thumb_proc, t16_push,       0b10110100000, 0b11111110000, 11);
    arm_proc_set(thumb_proc, t16_ror,        0b01000001110, 0b11111111110, 11);
    arm_proc_set(thumb_proc, t16_rsb_rdn3,   0b01000010010, 0b11111111110, 11);
    arm_proc_set(thumb_proc, t16_sbc_rdn3,   0b01000001100, 0b11111111110, 11);
    arm_proc_set(thumb_proc, t16_stm,        0b11000000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_str_imm5,   0b01100000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_str_sp8,    0b10010000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_str_reg,    0b01010000000, 0b11111110000, 11);
    arm_proc_set(thumb_proc, t16_strb_imm5,  0b01110000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_strb_reg,   0b01010100000, 0b11111110000, 11);
    arm_proc_set(thumb_proc, t16_strh_imm5,  0b10000000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_strh_reg,   0b01010010000, 0b11111110000, 11);
    arm_proc_set(thumb_proc, t16_sub_imm3,   0b00011110000, 0b11111110000, 11);
    arm_proc_set(thumb_proc, t16_sub_imm8,   0b00111000000, 0b11111000000, 11);
    arm_proc_set(thumb_proc, t16_sub_reg,    0b00011010000, 0b11111110000, 11);
    arm_proc_set(thumb_proc, t16_sub_sp7,    0b10110000100, 0b11111111100, 11);
    arm_proc_set(thumb_proc, t16_svc,        0b11011111100, 0b11111111000, 11);
    arm_proc_set(thumb_proc, t16_tst_rdn3,   0b01000010000, 0b11111111110, 11);
}

void arm_init() {
    // Initialize memory buffer contents, not the pointers themselves
    // as they are already declared as arrays
    memset(bios, 0, sizeof(bios));
    memset(wram_board, 0, ewram_size);
    memset(wram_chip, 0, sizeof(wram_chip));
    memset(palette_ram, 0, sizeof(palette_ram));
    memset(vram, 0, sizeof(vram));
    memset(oam, 0, sizeof(oam));

    // Initialize memory pointers - don't allocate arrays that are already defined.
    // flash (128 KB) is lazily allocated on first access since most ROMs don't
    // use flash saves; allocating it eagerly leaves no room for the ROM chunks.
    eeprom = malloc(0x2000);
    sram   = malloc(0x10000);
    flash  = NULL;

    arm_proc_init();
    thumb_proc_init();

    key_input.w = 0x3ff;
    wait_cnt.w  = 0;
    arm_cycles  = 0;

    update_ws();
}

void arm_uninit() {
    // Don't free statically allocated arrays
    // Only free dynamically allocated memory
    free(eeprom);
    free(sram);
    free(flash);
}

#define ARM_COND_UNCOND  0b1111

static inline void t16_inc_r15() {
    if (pipe_reload)
        pipe_reload = false;
    else
        arm_r.r[15] += 2;
}

static inline void t16_step() {
    arm_pipe[1] = arm_fetchh(SEQUENTIAL);

    // Inline fast paths for the most common Thumb opcodes that can be
    // identified by their top 5 bits alone. The handlers and their
    // helpers (t16_data_imm8_op, arm_memio_imm5_op, arm_arith_*,
    // arm_lsl/lsr/asr, arm_memio_ldr/str, etc.) are static inline,
    // so the compiler folds them in here, eliminating the function-call
    // overhead, the per-op operand decode call, and the indirect
    // dispatch through thumb_proc[]. Per-op savings are ~5-15 SH4A
    // cycles depending on the handler's body size; on Thumb-dominant
    // games (Minish Cap, Pokémon, etc.) the dispatch itself becomes
    // straight-line code with no function-call boundaries.
    //
    // Non-matching opcodes fall through to the existing function-pointer
    // dispatch.
    switch (arm_op >> 11) {
        case 0b00000: t16_lsl_imm5();  break;  // LSL  Rd, Rm, #imm5
        case 0b00001: t16_lsr_imm5();  break;  // LSR  Rd, Rm, #imm5
        case 0b00010: t16_asr_imm5();  break;  // ASR  Rd, Rm, #imm5
        case 0b00100: t16_mov_imm();   break;  // MOV  Rd, #imm8
        case 0b00101: t16_cmp_imm8();  break;  // CMP  Rd, #imm8
        case 0b00110: t16_add_imm8();  break;  // ADD  Rd, #imm8
        case 0b00111: t16_sub_imm8();  break;  // SUB  Rd, #imm8
        case 0b01001: t16_ldr_pc8();   break;  // LDR  Rd, [PC, #imm8] (constant pool)
        case 0b01100: t16_str_imm5();  break;  // STR  Rd, [Rn, #imm5]
        case 0b01101: t16_ldr_imm5();  break;  // LDR  Rd, [Rn, #imm5]
        case 0b10010: t16_str_sp8();   break;  // STR  Rd, [SP, #imm8]
        case 0b10011: t16_ldr_sp8();   break;  // LDR  Rd, [SP, #imm8]
        case 0b11100: t16_b_imm11();   break;  // B    #imm11 (uncond branch)
        default:      thumb_proc[arm_op >> 5]();
    }

    t16_inc_r15();
}

static inline void arm_inc_r15() {
    if (pipe_reload)
        pipe_reload = false;
    else
        arm_r.r[15] += 4;
}

static inline void arm_step() {
    arm_pipe[1] = arm_fetch(SEQUENTIAL);

    uint32_t proc;

    proc  = (arm_op >> 16) & 0xff0;
    proc |= (arm_op >>  4) & 0x00f;

    uint32_t cond = arm_op >> 28;

    // cond == 14 (AL = always) is by far the most common — most ARM code is
    // unconditionally executed at the per-instruction granularity. Skip the
    // arm_cond() call entirely for that case. cond == 15 is the
    // "unconditional" form (a few instructions like BLX have this encoding).
    //
    // Compressed dispatch: arm_proc_idx_*[proc] returns a 1-byte index
    // into arm_proc_handlers[] (the actual function pointer). See the
    // table-definition block for why this wins.
    if (cond == 14) {
        arm_proc_handlers[arm_proc_idx_0[proc]]();
    } else if (cond == ARM_COND_UNCOND) {
        arm_proc_handlers[arm_proc_idx_1[proc]]();
    } else if (arm_cond(cond)) {
        arm_proc_handlers[arm_proc_idx_0[proc]]();
    }

    arm_inc_r15();
}

// Sparse PC trace, sampled every Nth instruction. Lets the diagnostic dump
// show where execution is actually spending its time within one frame.
#define ARM_PC_TRACE_LEN 16
uint32_t arm_pc_trace[ARM_PC_TRACE_LEN];
uint32_t arm_pc_trace_pos;
uint32_t arm_pc_trace_skip;
volatile uint32_t arm_undef_count;
volatile uint32_t arm_undef_first_pc;
volatile uint32_t arm_undef_first_op;
volatile uint32_t arm_first_low_pc;
volatile uint32_t arm_first_low_lr;
volatile uint32_t arm_first_low_op;
volatile uint8_t  arm_first_low_was_thumb;
volatile uint8_t  arm_first_low_set;
volatile uint32_t arm_swi_count[256];
volatile uint8_t  arm_swi_last;

// Place the inner dispatch loop into ILRAM (4 KB on-chip code RAM, 0-wait,
// never evicted from I-cache). t16_step and arm_step are static inline and
// fold into arm_exec at -O3 -flto, as do the per-opcode fast paths in
// t16_step's switch — so the entire steady-state Thumb dispatch body lands
// in ILRAM. Out-of-line handlers reached via thumb_proc[]/arm_proc_handlers[]
// stay in .text. .ilram is 4 KB; arm_exec is ~1.5 KB after inlining, leaving
// headroom for placing more hot code (arm_check_irq, arm_load_pipe) later.
//
// We use .gint.mapped (gint's "ax" section also placed in ILRAM by the
// fxcg50 linker script) rather than the GILRAM macro's .ilram, because
// .ilram is already populated with data (e.g. gint's ILbuf) and putting
// code there would trigger a section-type conflict at link time.
//
// gint_set_onchip_save_mode(BACKUP) in main.c covers the entire ILRAM
// region across world switches, so chunk-load fread can still happen
// without trashing the dispatch code.
#define ILRAM_CODE __attribute__((section(".gint.mapped")))
void arm_exec(uint32_t target_cycles) ILRAM_CODE;
void arm_exec(uint32_t target_cycles) {
    if (int_halt) {
        timers_clock(target_cycles);

        return;
    }

    // Timer clocking is batched to once per arm_exec call instead of once
    // per emulated instruction. Saves a load+test+sub+fn-call per inner
    // iteration. Loss of granularity: timer overflows / IRQs that would
    // have fired mid-scanline now fire at the end of the scanline. Most
    // games tolerate this since they only check timers at VBlank anyway.
    uint32_t start_cycles = arm_cycles;

    while (arm_cycles < target_cycles) {
        arm_op      = arm_pipe[0];
        arm_pipe[0] = arm_pipe[1];

#ifdef ARM_TRACE_ENABLE
        // Per-instruction PC trace, sampled every 4096 instructions. Useful
        // for diagnosing where execution is spending its time, but the
        // increment + AND check costs a few SH4 cycles per emulated ARM
        // instruction. Compile with -DARM_TRACE_ENABLE to opt back in.
        if ((arm_pc_trace_skip++ & 0xfff) == 0) {
            arm_pc_trace[arm_pc_trace_pos & (ARM_PC_TRACE_LEN - 1)] = arm_r.r[15];
            arm_pc_trace_pos++;
        }
#endif

        if (arm_in_thumb())
            t16_step();
        else
            arm_step();

        if (int_halt) arm_cycles = target_cycles;
    }

    if (tmr_enb) timers_clock(arm_cycles - start_cycles);
    arm_cycles -= target_cycles;
}

void arm_int(uint32_t address, int8_t mode) {
    // Saving CPSR to SPSR: pack lazy NZCV first so the saved snapshot is correct.
    arm_flags_to_cpsr();
    uint32_t cpsr = arm_r.cpsr;

    arm_mode_set(mode);
    arm_spsr_set(cpsr);

    //Set FIQ Disable flag based on exception
    if (address == ARM_VEC_FIQ ||
        address == ARM_VEC_RESET)
        arm_flag_set(ARM_F, true);

    /*
     * Adjust PC based on exception
     *
     * Exc  ARM  Thumb
     * DA   $+8  $+8
     * FIQ  $+4  $+4
     * IRQ  $+4  $+4
     * PA   $+4  $+4
     * UND  $+4  $+2
     * SVC  $+4  $+2
     */
    if (address == ARM_VEC_UND ||
        address == ARM_VEC_SVC) {
        if (arm_in_thumb())
            arm_r.r[15] -= 2;
        else
            arm_r.r[15] -= 4;
    }

    if (address == ARM_VEC_FIQ ||
        address == ARM_VEC_IRQ ||
        address == ARM_VEC_PABT) {
        if (arm_in_thumb())
            arm_r.r[15] -= 0;
        else
            arm_r.r[15] -= 4;
    }

    arm_flag_set(ARM_T, false);
    arm_flag_set(ARM_I, true);

    arm_r.r[14] = arm_r.r[15];
    arm_r.r[15] = address;

    arm_load_pipe();
}

void arm_check_irq() {
    if (!arm_flag_tst(ARM_I) &&
        (int_enb_m.w & 1) &&
        (int_enb.w & int_ack.w)) {
        // Read the cart's installed user-IRQ handler from 0x03007FFC.
        // If non-zero, take the HLE'd entry path that skips the BIOS's
        // ~10-instruction prologue/epilogue. If zero, fall through to
        // the original arm_int(VEC_IRQ, IRQ) so the BIOS's null-check
        // / dispatcher can run as it would on real hardware (matters
        // during early-boot frames before the cart sets the pointer).
        uint32_t handler = arm_read(0x03007FFC);
        if (handler) {
            hle_irq_enter(handler);
            return;
        }
        arm_int(ARM_VEC_IRQ, ARM_IRQ);
    }
}

void arm_reset() {
    arm_int(ARM_VEC_RESET, ARM_SVC);

    // Bypass the BIOS boot path. The BIOS in use is a replacement
    // (Normmatt's or similar) which assumes the OS / boot loader has already
    // set up CPSR, the banked stack pointers, and POSTFLG before entry --
    // jumping to PC=0 with a zero state lands us in the IRQ-exit code at
    // 0x68 and corrupts CPSR within a frame. Real cartridges don't depend
    // on the boot animation, so set up the standard GBA banked stack
    // pointers and start execution directly at the cart entry point.
    arm_r.r13_svc = 0x03007FE0;
    arm_r.r13_irq = 0x03007FA0;
    arm_r.r13_usr = 0x03007F00;
    arm_r.r[13]   = 0x03007F00;  // currently-active SP (we're in SVC, but
                                  // most carts switch to USR/SYS quickly)

    // POSTFLG stays 0 (cold boot). This particular replacement BIOS treats
    // POSTFLG==1 in its reset path (0x68 onwards) as "we are inside an
    // exception handler -- jump to the shared handler at 0x1C". When the
    // cart's IRQ handler at 0x03007FFC is uninitialised, the BIOS IRQ entry
    // dereferences NULL, lands at 0x0 (reset), follows that POSTFLG==1
    // branch, and ends up looping in the shared handler instead of running
    // cart code. Leaving POSTFLG at 0 keeps the BIOS reset path on its
    // cold-boot fall-through.
    extern uint8_t post_boot;
    post_boot = 0;

    arm_r.r[15] = 0x08000000;
    arm_load_pipe();
}