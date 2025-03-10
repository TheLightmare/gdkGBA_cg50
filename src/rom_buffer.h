#ifndef ROM_BUFFER_H
#define ROM_BUFFER_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// Size of each ROM chunk in the buffer (e.g., 64KB)
#define ROM_CHUNK_SIZE (64 * 1024)
// Number of chunks to keep in memory at once
#define ROM_BUFFER_CHUNKS 4

typedef struct {
    FILE* rom_file;           // File handle for the ROM
    uint32_t rom_size;        // Total size of the ROM
    uint32_t rom_mask;        // ROM size mask for wrapping
    
    // Buffer data
    uint8_t* chunk_buffers[ROM_BUFFER_CHUNKS];
    uint32_t chunk_addresses[ROM_BUFFER_CHUNKS];  // Start address of each chunk
    int8_t chunk_lru[ROM_BUFFER_CHUNKS];         // LRU counter for each chunk
} RomBuffer;

// Initialize the ROM buffer system
bool rom_buffer_init(RomBuffer* buffer, const char* filename, uint32_t rom_mask);

// Clean up resources
void rom_buffer_cleanup(RomBuffer* buffer);

// Read a byte from the ROM at the specified address
uint8_t rom_buffer_read_8(RomBuffer* buffer, uint32_t address);

// Read a halfword (16 bits) from the ROM at the specified address
uint16_t rom_buffer_read_16(RomBuffer* buffer, uint32_t address);

// Read a word (32 bits) from the ROM at the specified address
uint32_t rom_buffer_read_32(RomBuffer* buffer, uint32_t address);

#endif // ROM_BUFFER_H
