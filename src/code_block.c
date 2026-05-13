#include "code_block.h"

uint16_t code_page_gen[CODE_TOTAL_PAGES];

void code_block_invalidate_page(uint32_t addr) {
    uint16_t idx = code_page_idx(addr);
    if (idx != CODE_PAGE_NONE) {
        code_page_gen[idx]++;
    }
}
