#include "rom_buffer.h"
#include <stdlib.h>
#include <string.h>

// Loads a chunk of the ROM into a buffer
static bool load_chunk(RomBuffer* buffer, int chunk_idx, uint32_t chunk_address) {
    // Calculate the actual file offset for this chunk
    uint32_t file_offset = chunk_address & buffer->rom_mask;
    
    // Set the file position
    if (fseek(buffer->rom_file, file_offset, SEEK_SET) != 0) {
        return false;
    }
    
    // Read the chunk from the file
    uint32_t bytes_to_read = ROM_CHUNK_SIZE;
    if (file_offset + bytes_to_read > buffer->rom_size) {
        bytes_to_read = buffer->rom_size - file_offset;
    }
    
    size_t bytes_read = fread(buffer->chunk_buffers[chunk_idx], 1, bytes_to_read, buffer->rom_file);
    if (bytes_read != bytes_to_read) {
        return false;
    }
    
    // Clear any remaining bytes in the buffer if we read less than the chunk size
    if (bytes_read < ROM_CHUNK_SIZE) {
        memset(buffer->chunk_buffers[chunk_idx] + bytes_read, 0, ROM_CHUNK_SIZE - bytes_read);
    }
    
    // Update the chunk address
    buffer->chunk_addresses[chunk_idx] = chunk_address;
    
    return true;
}

// Find the chunk containing the specified address, load it if necessary
static int get_chunk_for_address(RomBuffer* buffer, uint32_t address) {
    uint32_t chunk_address = address & ~(ROM_CHUNK_SIZE - 1);
    int lru_chunk = 0;
    int lru_value = buffer->chunk_lru[0];
    
    // First, check if the chunk is already loaded
    for (int i = 0; i < ROM_BUFFER_CHUNKS; i++) {
        if (buffer->chunk_addresses[i] == chunk_address) {
            // Found it! Update LRU counters
            buffer->chunk_lru[i] = ROM_BUFFER_CHUNKS;
            for (int j = 0; j < ROM_BUFFER_CHUNKS; j++) {
                if (j != i && buffer->chunk_lru[j] > 0) {
                    buffer->chunk_lru[j]--;
                }
            }
            return i;
        }
        
        // Track the least recently used chunk
        if (buffer->chunk_lru[i] < lru_value) {
            lru_value = buffer->chunk_lru[i];
            lru_chunk = i;
        }
    }
    
    // Chunk not found, replace the least recently used one
    if (!load_chunk(buffer, lru_chunk, chunk_address)) {
        return -1;  // Failed to load chunk
    }
    
    // Update LRU counters
    buffer->chunk_lru[lru_chunk] = ROM_BUFFER_CHUNKS;
    for (int j = 0; j < ROM_BUFFER_CHUNKS; j++) {
        if (j != lru_chunk && buffer->chunk_lru[j] > 0) {
            buffer->chunk_lru[j]--;
        }
    }
    
    return lru_chunk;
}

bool rom_buffer_init(RomBuffer* buffer, const char* filename, uint32_t rom_mask) {
    // Open the ROM file
    buffer->rom_file = fopen(filename, "rb");
    if (!buffer->rom_file) {
        return false;
    }
    
    // Get the ROM size
    fseek(buffer->rom_file, 0, SEEK_END);
    buffer->rom_size = ftell(buffer->rom_file);
    fseek(buffer->rom_file, 0, SEEK_SET);
    buffer->rom_mask = rom_mask;
    
    // Initialize buffers and LRU tracking
    for (int i = 0; i < ROM_BUFFER_CHUNKS; i++) {
        buffer->chunk_buffers[i] = (uint8_t*)malloc(ROM_CHUNK_SIZE);
        if (!buffer->chunk_buffers[i]) {
            // Clean up on failure
            for (int j = 0; j < i; j++) {
                free(buffer->chunk_buffers[j]);
            }
            fclose(buffer->rom_file);
            return false;
        }
        
        buffer->chunk_addresses[i] = 0xFFFFFFFF;  // Invalid address
        buffer->chunk_lru[i] = i;  // Initialize LRU values
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
            free(buffer->chunk_buffers[i]);
            buffer->chunk_buffers[i] = NULL;
        }
    }
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
    uint16_t value;
    value = rom_buffer_read_8(buffer, address);
    value |= rom_buffer_read_8(buffer, address + 1) << 8;
    return value;
}

uint32_t rom_buffer_read_32(RomBuffer* buffer, uint32_t address) {
    uint32_t value;
    value = rom_buffer_read_8(buffer, address);
    value |= rom_buffer_read_8(buffer, address + 1) << 8;
    value |= rom_buffer_read_8(buffer, address + 2) << 16;
    value |= rom_buffer_read_8(buffer, address + 3) << 24;
    return value;
}
