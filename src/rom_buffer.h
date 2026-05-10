#ifndef ROM_BUFFER_H
#define ROM_BUFFER_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// Per-chunk size and chunk count. Total cache memory = SIZE × COUNT and
// must fit in the extram budget (~2.75 MB free after the 256 KB EWRAM
// allocation, on CG-50 with the standard OS).
//
// Tuning history:
//   - 64 KB × 8 chunks (512 KB): heavy thrashing during Zelda init.
//   - 64 KB × 32 chunks (2 MB): default for most prior testing.
//   - 64 KB × 44 chunks (2.75 MB): no measurable improvement; benchmarks
//     showed 939 chunk_miss across 240 frames in active gameplay,
//     averaging ~4/frame.
//   - 32 KB × 88 chunks (2.75 MB, current): same total cache but twice
//     as many distinct ROM regions resident. For a 14 MB cart with
//     spread-thin access (many small accesses across many regions, as
//     opposed to one big sequential read), finer granularity reduces
//     thrashing materially: a "miss" pulls only 32 KB instead of 64 KB,
//     halving the per-miss fread time.
//
// rom_buffer_init() accepts partial allocation >= 2, so if extram is
// tighter than expected we degrade gracefully to fewer chunks.
//
// Cost of more chunks:
//   - get_chunk_for_address loops up to ROM_BUFFER_CHUNKS on a miss.
//     The last-hit cache makes the hit path O(1), and even 88 iterations
//     of compare-and-branch is negligible compared to the storage I/O
//     a miss represents.
//   - RomBuffer struct grows by ~9 bytes per chunk (ptr+addr+lru) — at
//     88 chunks the struct is ~800 bytes, still trivially fits in XYRAM.
#define ROM_CHUNK_SIZE     (32 * 1024)
#define ROM_BUFFER_CHUNKS  88

typedef struct {
    FILE* rom_file;           // File handle for the ROM
    uint32_t rom_size;        // Total size of the ROM
    uint32_t rom_mask;        // ROM size mask for wrapping

    // Number of chunks actually allocated (may be < ROM_BUFFER_CHUNKS if
    // memory was tight). All loops over chunks use this, not the macro.
    int active_chunks;

    // Buffer data
    uint8_t* chunk_buffers[ROM_BUFFER_CHUNKS];
    uint32_t chunk_addresses[ROM_BUFFER_CHUNKS];  // Start address of each chunk
    int8_t chunk_lru[ROM_BUFFER_CHUNKS];         // LRU counter for each chunk

    // Hot-path cache: the last chunk address served, and its index. Lets the
    // common "same chunk as last access" hit skip the LRU loop entirely.
    uint32_t last_chunk_address;
    int8_t   last_chunk_idx;
} RomBuffer;

// Failure reason returned by rom_buffer_init via the optional out-param.
typedef enum {
    ROM_INIT_OK = 0,
    ROM_INIT_FOPEN_FAILED,
    ROM_INIT_MALLOC_FAILED
} rom_init_status_e;

// Initialize the ROM buffer system. If failed_chunk is non-NULL it is set to
// the index (0..ROM_BUFFER_CHUNKS-1) of the chunk whose malloc failed when
// status == ROM_INIT_MALLOC_FAILED; otherwise to -1.
bool rom_buffer_init(RomBuffer* buffer, const char* filename, uint32_t rom_mask,
                     rom_init_status_e* status, int* failed_chunk);

// Clean up resources
void rom_buffer_cleanup(RomBuffer* buffer);

// Read a byte from the ROM at the specified address
uint8_t rom_buffer_read_8(RomBuffer* buffer, uint32_t address);

// Read a halfword (16 bits) from the ROM at the specified address
uint16_t rom_buffer_read_16(RomBuffer* buffer, uint32_t address);

// Read a word (32 bits) from the ROM at the specified address
uint32_t rom_buffer_read_32(RomBuffer* buffer, uint32_t address);

// Inline fast-path variants for instruction fetch. Skip the function call
// to rom_buffer_read_* and the inner get_chunk_for_address dispatch when
// the access lands in the same chunk as the previous fetch — the
// overwhelmingly common case in any game's hot loop.
//
// Caller invariant: address is aligned (2-byte for _16, 4-byte for _32).
// Aligned reads in 64 KB chunks never span a chunk boundary so we don't
// need the per-byte assembly that the slow path uses near boundaries.
//
// Bus-traffic optimization: on big-endian hosts (SH4A on the CG50)
// chunks are stored byteswapped at load time (see load_chunk in rom_buffer.c).
// That means a native 32-bit aligned load directly yields the GBA's
// little-endian-decoded word — one bus transaction instead of four byte
// loads + shifts/ORs. On a little-endian host, no swap is needed and a
// native load gives the same result. The only address munging is for
// sub-word reads on BE hosts: byte at original offset N is at swapped
// offset N^3, halfword at N is at swapped offset N^2.
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#  define ROM_BYTE_OFF_XOR  3u
#  define ROM_HALF_OFF_XOR  2u
#else
#  define ROM_BYTE_OFF_XOR  0u
#  define ROM_HALF_OFF_XOR  0u
#endif

static inline uint16_t rom_buffer_read_16_fast(RomBuffer *buffer,
                                               uint32_t address)
{
    if (__builtin_expect(
            (address & ~(ROM_CHUNK_SIZE - 1)) == buffer->last_chunk_address &&
            buffer->last_chunk_idx >= 0, 1)) {
        const uint8_t *p = buffer->chunk_buffers[buffer->last_chunk_idx] +
                           ((address & (ROM_CHUNK_SIZE - 1)) ^ ROM_HALF_OFF_XOR);
        return *(const uint16_t *)p;
    }
    return rom_buffer_read_16(buffer, address);
}

static inline uint32_t rom_buffer_read_32_fast(RomBuffer *buffer,
                                               uint32_t address)
{
    if (__builtin_expect(
            (address & ~(ROM_CHUNK_SIZE - 1)) == buffer->last_chunk_address &&
            buffer->last_chunk_idx >= 0, 1)) {
        const uint8_t *p = buffer->chunk_buffers[buffer->last_chunk_idx] +
                           (address & (ROM_CHUNK_SIZE - 1));
        return *(const uint32_t *)p;
    }
    return rom_buffer_read_32(buffer, address);
}

#endif // ROM_BUFFER_H
