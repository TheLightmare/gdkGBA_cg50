#include "rom_buffer.h"
#include <stdlib.h>
#include <string.h>
#include <gint/gint.h>
#include <gint/kmalloc.h>

#include "bench.h"
#include "build_flags.h"
#include "extram.h"

// Allocate a chunk buffer. Prefers the optional "extram" arena (3 MB on
// CG-50 OS <= 03.06) so we leave _uram free for sram/eeprom/flash and
// other small allocations. Falls back to plain malloc when extram is
// unavailable.
static void *chunk_alloc(void) {
    if (extram_available) {
        void *p = kmalloc(ROM_CHUNK_SIZE, "extram");
        if (p) return p;
    }
    return malloc(ROM_CHUNK_SIZE);
}

static void chunk_free(void *p) {
    // kfree() routes back to whichever arena owned the allocation, so it
    // works for both extram and the default heap.
    kfree(p);
}

// File I/O for chunk loading runs inside a gint_world_switch. Calls into
// the Fugue filesystem (especially fseek/fread on fragmented files) rely on
// OS-world state that isn't available while gint is in control of the
// hardware. With contiguous (defragmented) files the OS tolerates being
// called from gint world; with fragmented files, the fragment-table
// lookup panics the calculator. The world switch is the documented fix.
struct chunk_io_args {
    FILE   *file;
    long    offset;
    void   *buffer;
    size_t  bytes;
    size_t  result;
};

static int do_chunk_io(struct chunk_io_args *a) {
    if (fseek(a->file, a->offset, SEEK_SET) != 0) {
        a->result = 0;
        return -1;
    }
    a->result = fread(a->buffer, 1, a->bytes, a->file);
    return 0;
}

// Big-endian-host post-load step: chunks come off disk in little-endian
// (GBA storage order). To let arm_fetch / rom_buffer_read_*_fast use
// native 32-bit loads — one bus transaction instead of four byte loads
// plus shifts/ORs — we byteswap each 4-byte word in place. After this,
// a native BE word load at offset 0 of any aligned word yields exactly
// the LE-decoded value the guest expects.
//
// One-time cost per chunk_miss (~1/frame steady state, ~0.3 ms on a 32 KB
// chunk at 117 MHz Bphi); amortized over thousands of fetches.
//
// On little-endian hosts this is a no-op: native loads already match the
// GBA storage order.
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
static void byteswap_chunk(uint8_t *buf) {
    uint32_t *w = (uint32_t *)buf;
    for (uint32_t i = 0; i < ROM_CHUNK_SIZE / 4; i++) {
        w[i] = __builtin_bswap32(w[i]);
    }
}
#endif

// Loads a chunk of the ROM into a buffer
static bool load_chunk(RomBuffer* buffer, int chunk_idx, uint32_t chunk_address) {
    uint32_t file_offset = chunk_address & buffer->rom_mask;

    uint32_t bytes_to_read = ROM_CHUNK_SIZE;
    if (file_offset + bytes_to_read > buffer->rom_size) {
        bytes_to_read = buffer->rom_size - file_offset;
    }

    struct chunk_io_args args = {
        .file   = buffer->rom_file,
        .offset = (long)file_offset,
        .buffer = buffer->chunk_buffers[chunk_idx],
        .bytes  = bytes_to_read,
        .result = 0
    };
    gint_world_switch(GINT_CALL(do_chunk_io, (void *)&args));

    if (args.result != bytes_to_read) {
        return false;
    }

    // Zero-fill any remaining bytes if the file was shorter than chunk size.
    if (args.result < ROM_CHUNK_SIZE) {
        memset((uint8_t *)buffer->chunk_buffers[chunk_idx] + args.result,
               0, ROM_CHUNK_SIZE - args.result);
    }

#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    // Byteswap whole chunk (zero-fill bytes byteswap to zero, no-op).
    byteswap_chunk(buffer->chunk_buffers[chunk_idx]);
#endif

    buffer->chunk_addresses[chunk_idx] = chunk_address;

    return true;
}

// Find the chunk containing the specified address, load it if necessary.
//
// Lookup order:
//   1. last_chunk fast path (handles >99% of accesses in the hot loop).
//   2. Direct hash table (O(1), maintained alongside chunk loads/evictions).
//   3. Linear scan of active chunks (handles hash collisions and any case
//      where a chunk is in the cache but its table slot got displaced).
//      Also tracks the LRU candidate so a true miss doesn't pay a second
//      pass.
//   4. On a true miss: evict the LRU chunk, fread the new one, refresh
//      both the table and last_chunk pointers.
static int get_chunk_for_address(RomBuffer* buffer, uint32_t address) {
    uint32_t chunk_address = address & ~(ROM_CHUNK_SIZE - 1);

    // 1. Hot path: same chunk as the previous access. No bookkeeping.
    if (chunk_address == buffer->last_chunk_address && buffer->last_chunk_idx >= 0)
        return buffer->last_chunk_idx;

    // 2. Direct hash lookup. Fresh entries are authoritative; stale entries
    //    fall through to the linear scan, which refreshes the table.
    uint32_t h = rom_chunk_hash(chunk_address);
    RomLookupEntry *e = &buffer->chunk_lookup[h];
    if (e->chunk_addr == chunk_address && e->chunk_idx >= 0) {
        int i = e->chunk_idx;
        if (buffer->chunk_addresses[i] == chunk_address) {
            buffer->chunk_lru_ts[i] = ++buffer->lru_clock;
            buffer->last_chunk_address = chunk_address;
            buffer->last_chunk_idx = i;
            return i;
        }
        // Stale: chunk_addresses[i] no longer holds chunk_address. Fall
        // through; the linear scan will either find it elsewhere or
        // declare a true miss.
    }

    // 3. Linear-scan fallback. Single pass: locate the chunk OR the LRU
    //    eviction target. Empty slots (chunk_addresses[i] == 0xFFFFFFFF)
    //    are picked up as natural eviction targets via their initial LRU
    //    timestamp of 0.
    int active = buffer->active_chunks;
    int lru_idx = 0;
    uint32_t lru_ts = buffer->chunk_lru_ts[0];

    for (int i = 0; i < active; i++) {
        uint32_t addr_i = buffer->chunk_addresses[i];
        if (addr_i == chunk_address) {
            buffer->chunk_lru_ts[i] = ++buffer->lru_clock;
            buffer->last_chunk_address = chunk_address;
            buffer->last_chunk_idx = i;
            // Refresh the table -- might have been displaced by a hash
            // collision, or this chunk was loaded before its slot was
            // owned by it.
            e->chunk_addr = chunk_address;
            e->chunk_idx = (int16_t)i;
            return i;
        }
        uint32_t ts_i = buffer->chunk_lru_ts[i];
        if (ts_i < lru_ts) {
            lru_ts = ts_i;
            lru_idx = i;
        }
    }

    // 4. True miss. Evict the LRU chunk and load the new one.
    BENCH_INC(bench_chunk_miss);

    // Invalidate the table entry that pointed at the chunk we're about to
    // overwrite, but only if it still owns this slot (a later access to a
    // colliding chunk may already have stolen it -- in which case there's
    // nothing for us to clean up).
    uint32_t evicting_addr = buffer->chunk_addresses[lru_idx];
    if (evicting_addr != 0xFFFFFFFFu) {
        uint32_t old_h = rom_chunk_hash(evicting_addr);
        RomLookupEntry *old_e = &buffer->chunk_lookup[old_h];
        if (old_e->chunk_idx == lru_idx && old_e->chunk_addr == evicting_addr) {
            old_e->chunk_addr = 0xFFFFFFFFu;
            old_e->chunk_idx = -1;
        }
    }

    if (!load_chunk(buffer, lru_idx, chunk_address)) {
        return -1;  // Failed to load chunk
    }

    buffer->chunk_lru_ts[lru_idx] = ++buffer->lru_clock;
    buffer->last_chunk_address = chunk_address;
    buffer->last_chunk_idx = lru_idx;

    // Install in the table. May overwrite an entry for a different chunk
    // that hashes here; that older chunk stays reachable via linear scan
    // and will refresh its own entry on next access.
    e->chunk_addr = chunk_address;
    e->chunk_idx = (int16_t)lru_idx;

    return lru_idx;
}

bool rom_buffer_init(RomBuffer* buffer, const char* filename, uint32_t rom_mask,
                     rom_init_status_e* status, int* failed_chunk) {
    if (status) *status = ROM_INIT_OK;
    if (failed_chunk) *failed_chunk = -1;

    buffer->rom_file = fopen(filename, "rb");
    if (!buffer->rom_file) {
        if (status) *status = ROM_INIT_FOPEN_FAILED;
        return false;
    }

    fseek(buffer->rom_file, 0, SEEK_END);
    buffer->rom_size = ftell(buffer->rom_file);
    fseek(buffer->rom_file, 0, SEEK_SET);
    buffer->rom_mask = rom_mask;
    buffer->last_chunk_address = 0xFFFFFFFF;
    buffer->last_chunk_idx = -1;
    buffer->active_chunks = 0;
    buffer->lru_clock = 0;

    // Empty out the direct-lookup table.
    for (int i = 0; i < ROM_LOOKUP_SIZE; i++) {
        buffer->chunk_lookup[i].chunk_addr = 0xFFFFFFFFu;
        buffer->chunk_lookup[i].chunk_idx  = -1;
        buffer->chunk_lookup[i]._pad       = 0;
    }

    // Allocate chunks. Each one prefers extram, falls back to default heap.
    // If we can't get the full ROM_BUFFER_CHUNKS, accept any count >= 2 --
    // a smaller cache hurts performance but is functional. Below 2 there's
    // no point continuing.
    for (int i = 0; i < ROM_BUFFER_CHUNKS; i++) {
        buffer->chunk_buffers[i] = (uint8_t *)chunk_alloc();
        if (!buffer->chunk_buffers[i]) {
            if (i >= 2) {
                // Partial success -- shrink active set and continue.
                break;
            }
            // Hard failure with too few chunks. Tear down.
            for (int j = 0; j < i; j++) chunk_free(buffer->chunk_buffers[j]);
            fclose(buffer->rom_file);
            buffer->rom_file = NULL;
            if (status) *status = ROM_INIT_MALLOC_FAILED;
            if (failed_chunk) *failed_chunk = i;
            return false;
        }

        buffer->chunk_addresses[i] = 0xFFFFFFFF;
        // Initial timestamp 0 -- empty slots are picked up first as
        // eviction targets without needing any special-case code.
        buffer->chunk_lru_ts[i] = 0;
        buffer->active_chunks = i + 1;
    }

    return true;
}

void rom_buffer_cleanup(RomBuffer* buffer) {
    if (buffer->rom_file) {
        fclose(buffer->rom_file);
        buffer->rom_file = NULL;
    }

    for (int i = 0; i < ROM_BUFFER_CHUNKS; i++) {
        if (buffer->chunk_buffers[i]) {
            chunk_free(buffer->chunk_buffers[i]);
            buffer->chunk_buffers[i] = NULL;
        }
    }
    buffer->active_chunks = 0;
}

// Address munging is defined in rom_buffer.h alongside the byteswap
// convention so the inline _fast variants and the slow paths agree.
// On BE hosts: byte offset N is at swapped offset N^3, halfword offset N
// is at swapped offset N^2. On LE hosts both XOR masks are 0.

uint8_t rom_buffer_read_8(RomBuffer* buffer, uint32_t address) {
    int chunk_idx = get_chunk_for_address(buffer, address);
    if (chunk_idx < 0) {
        return 0xFF;  // Failed to load chunk
    }

    uint32_t offset = (address & (ROM_CHUNK_SIZE - 1)) ^ ROM_BYTE_OFF_XOR;
    return buffer->chunk_buffers[chunk_idx][offset];
}

uint16_t rom_buffer_read_16(RomBuffer* buffer, uint32_t address) {
    int chunk_idx = get_chunk_for_address(buffer, address);
    if (chunk_idx < 0) return 0xFFFF;

    uint32_t offset = address & (ROM_CHUNK_SIZE - 1);

    // Fast path: both bytes lie within the same chunk. On aligned reads
    // (the only kind callers issue) this is the only path.
    if (offset <= ROM_CHUNK_SIZE - 2) {
        const uint8_t *p = buffer->chunk_buffers[chunk_idx] +
                           (offset ^ ROM_HALF_OFF_XOR);
        return *(const uint16_t *)p;
    }

    // Straddling: fall back to per-byte fetch (also handles cross-chunk).
    return rom_buffer_read_8(buffer, address) |
           (rom_buffer_read_8(buffer, address + 1) << 8);
}

uint32_t rom_buffer_read_32(RomBuffer* buffer, uint32_t address) {
    int chunk_idx = get_chunk_for_address(buffer, address);
    if (chunk_idx < 0) return 0xFFFFFFFF;

    uint32_t offset = address & (ROM_CHUNK_SIZE - 1);

    if (offset <= ROM_CHUNK_SIZE - 4) {
        const uint8_t *p = buffer->chunk_buffers[chunk_idx] + offset;
        return *(const uint32_t *)p;
    }

    return  (uint32_t)rom_buffer_read_8(buffer, address)            |
           ((uint32_t)rom_buffer_read_8(buffer, address + 1) <<  8) |
           ((uint32_t)rom_buffer_read_8(buffer, address + 2) << 16) |
           ((uint32_t)rom_buffer_read_8(buffer, address + 3) << 24);
}
