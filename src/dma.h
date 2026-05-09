#ifndef DMA_H
#define DMA_H

#define DMA_REP  (1 <<  9)
#define DMA_32   (1 << 10)
#define DMA_IRQ  (1 << 14)
#define DMA_ENB  (1 << 15)

typedef enum {
    IMMEDIATELY = 0,
    VBLANK      = 1,
    HBLANK      = 2,
    SPECIAL     = 3
} dma_timing_e;

extern uint32_t dma_src_addr[4];
extern uint32_t dma_dst_addr[4];

extern uint32_t dma_count[4];

void dma_transfer_gba(dma_timing_e timing);
void dma_transfer_gba_fifo(uint8_t ch);

#endif