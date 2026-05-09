#ifndef ROM_BUFFER_H
#define ROM_BUFFER_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// Size of each ROM chunk in the buffer (e.g., 64KB)
#define ROM_CHUNK_SIZE (64 * 1024)
// Number of chunks to keep in memory at once. With the optional "extram"
// arena registered (3 MB on CG-50 OS <= 03.06), we have plenty of room and
// can cache enough cart code/data to nearly eliminate runtime chunk loads
// for most games. 8 chunks = 512 KB cache, large enough to hold the cart's
// hot code section plus a meaningful slice of its data tables.
//
// rom_buffer_init() prefers kmalloc(...,"extram") for the chunk buffers
// when available and falls back to default malloc otherwise. If extram
// isn't registered (e.g. newer OS), the fallback only has ~196 KB of
// _uram free, so we'd want a smaller value. The malloc loop in
// rom_buffer_init() degrades gracefully and reports how many chunks
// actually allocated.
#define ROM_BUFFER_CHUNKS 8

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

#endif // ROM_BUFFER_H
