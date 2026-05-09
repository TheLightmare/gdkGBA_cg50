#include "rom_buffer.h"
#include <stdlib.h>
#include <string.h>
#include <gint/gint.h>
#include <gint/kmalloc.h>

#include "bench.h"
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

    buffer->chunk_addresses[chunk_idx] = chunk_address;

    return true;
}

// Find the chunk containing the specified address, load it if necessary
static int get_chunk_for_address(RomBuffer* buffer, uint32_t address) {
    uint32_t chunk_address = address & ~(ROM_CHUNK_SIZE - 1);

    // Hot path: same chunk as the previous access. No LRU bookkeeping.
    if (chunk_address == buffer->last_chunk_address && buffer->last_chunk_idx >= 0)
        return buffer->last_chunk_idx;

    int active = buffer->active_chunks;
    int lru_chunk = 0;
    int lru_value = buffer->chunk_lru[0];

    // First, check if the chunk is already loaded
    for (int i = 0; i < active; i++) {
        if (buffer->chunk_addresses[i] == chunk_address) {
            // Found it! Update LRU counters
            buffer->chunk_lru[i] = active;
            for (int j = 0; j < active; j++) {
                if (j != i && buffer->chunk_lru[j] > 0) {
                    buffer->chunk_lru[j]--;
                }
            }
            buffer->last_chunk_address = chunk_address;
            buffer->last_chunk_idx = i;
            return i;
        }

        // Track the least recently used chunk
        if (buffer->chunk_lru[i] < lru_value) {
            lru_value = buffer->chunk_lru[i];
            lru_chunk = i;
        }
    }

    // Chunk not found, replace the least recently used one
    bench_chunk_miss++;
    if (!load_chunk(buffer, lru_chunk, chunk_address)) {
        return -1;  // Failed to load chunk
    }

    // Update LRU counters
    buffer->chunk_lru[lru_chunk] = active;
    for (int j = 0; j < active; j++) {
        if (j != lru_chunk && buffer->chunk_lru[j] > 0) {
            buffer->chunk_lru[j]--;
        }
    }

    buffer->last_chunk_address = chunk_address;
    buffer->last_chunk_idx = lru_chunk;
    return lru_chunk;
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
        buffer->chunk_lru[i] = i;
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

uint8_t rom_buffer_read_8(RomBuffer* buffer, uint32_t address) {
    int chunk_idx = get_chunk_for_address(buffer, address);
    if (chunk_idx < 0) {
        return 0xFF;  // Failed to load chunk
    }
    
    uint32_t offset = address & (ROM_CHUNK_SIZE - 1);
    return buffer->chunk_buffers[chunk_idx][offset];
}

uint16_t rom_buffer_read_16(RomBuffer* buffer, uint32_t address) {
    int chunk_idx = get_chunk_for_address(buffer, address);
    if (chunk_idx < 0) return 0xFFFF;

    uint32_t offset = address & (ROM_CHUNK_SIZE - 1);

    // Fast path: both bytes lie within the same chunk.
    if (offset <= ROM_CHUNK_SIZE - 2) {
        const uint8_t *p = buffer->chunk_buffers[chunk_idx] + offset;
        return p[0] | (p[1] << 8);
    }

    // Straddling: fall back to per-byte fetch.
    return rom_buffer_read_8(buffer, address) |
           (rom_buffer_read_8(buffer, address + 1) << 8);
}

uint32_t rom_buffer_read_32(RomBuffer* buffer, uint32_t address) {
    int chunk_idx = get_chunk_for_address(buffer, address);
    if (chunk_idx < 0) return 0xFFFFFFFF;

    uint32_t offset = address & (ROM_CHUNK_SIZE - 1);

    if (offset <= ROM_CHUNK_SIZE - 4) {
        const uint8_t *p = buffer->chunk_buffers[chunk_idx] + offset;
        return  (uint32_t)p[0]        |
               ((uint32_t)p[1] <<  8) |
               ((uint32_t)p[2] << 16) |
               ((uint32_t)p[3] << 24);
    }

    return  (uint32_t)rom_buffer_read_8(buffer, address)            |
           ((uint32_t)rom_buffer_read_8(buffer, address + 1) <<  8) |
           ((uint32_t)rom_buffer_read_8(buffer, address + 2) << 16) |
           ((uint32_t)rom_buffer_read_8(buffer, address + 3) << 24);
}
