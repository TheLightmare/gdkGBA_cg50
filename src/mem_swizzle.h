#ifndef MEM_SWIZZLE_H
#define MEM_SWIZZLE_H

#include <stdint.h>

// Host-endian-aware accessors for guest LE memory stored in host-native
// byte order. Used for EWRAM and IWRAM, where storing the buffer in
// host-native order lets aligned 16/32-bit reads use a single SH4 bus
// transaction instead of four byte loads + shift/OR assembly.
//
// Convention: the buffer holds the same logical bytes the GBA put there,
// but in a layout that's friendly to the host's natural integer load
// instructions. On a big-endian host (SH4 on the CG50), bytes within
// each 32-bit word are reversed; on a little-endian host (x86 dev
// builds), the layout matches GBA storage and the helpers are bare
// native loads.
//
// Argument convention: `off` is the offset the guest perceives -- a flat
// 0..mask byte offset into the region's logical view. Callers mask the
// guest address to the region size before passing it in.

#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#  define MEM_BYTE_OFF_XOR  3u
#  define MEM_HALF_OFF_XOR  2u
#else
#  define MEM_BYTE_OFF_XOR  0u
#  define MEM_HALF_OFF_XOR  0u
#endif

static inline uint8_t mem_swz_read_b(const uint8_t *buf, uint32_t off) {
    return buf[off ^ MEM_BYTE_OFF_XOR];
}

static inline uint16_t mem_swz_read_h(const uint8_t *buf, uint32_t off) {
    return *(const uint16_t *)(buf + ((off & ~1u) ^ MEM_HALF_OFF_XOR));
}

static inline uint32_t mem_swz_read_w(const uint8_t *buf, uint32_t off) {
    return *(const uint32_t *)(buf + (off & ~3u));
}

static inline void mem_swz_write_b(uint8_t *buf, uint32_t off, uint8_t v) {
    buf[off ^ MEM_BYTE_OFF_XOR] = v;
}

static inline void mem_swz_write_h(uint8_t *buf, uint32_t off, uint16_t v) {
    *(uint16_t *)(buf + ((off & ~1u) ^ MEM_HALF_OFF_XOR)) = v;
}

static inline void mem_swz_write_w(uint8_t *buf, uint32_t off, uint32_t v) {
    *(uint32_t *)(buf + (off & ~3u)) = v;
}

// Lvalue access for byte-level read-modify-write (e.g. `&= ~mask`). Use
// sparingly -- prefer the function-style helpers above when the code is
// a plain read or write.
#define MEM_SWZ_BYTE_LV(buf, off)  ((buf)[(off) ^ MEM_BYTE_OFF_XOR])

#endif // MEM_SWIZZLE_H
