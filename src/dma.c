#include "arm.h"
#include "arm_mem.h"

#include "dma.h"
#include "io.h"

//EXTERN VARIABLES DECLARATION ===

uint32_t dma_src_addr[4];
uint32_t dma_dst_addr[4];

uint32_t dma_count[4];

//================================

//TODO: Timing - DMA should take some amount of cycles

void dma_transfer_gba(dma_timing_e timing) {
    uint8_t ch;

    for (ch = 0; ch < 4; ch++) {
        if (!(dma_ch[ch].ctrl.w & DMA_ENB) ||
            ((dma_ch[ch].ctrl.w >> 12) & 3) != timing)
            continue;

        if (ch == 3)
            eeprom_idx = 0;

        int8_t unit_size = (dma_ch[ch].ctrl.w & DMA_32) ? 4 : 2;

        bool dst_reload = false;

        int8_t dst_inc = 0;
        int8_t src_inc = 0;

        switch ((dma_ch[ch].ctrl.w >> 5) & 3) {
            case 0: dst_inc =  unit_size; break;
            case 1: dst_inc = -unit_size; break;
            case 3:
                dst_inc = unit_size;
                dst_reload = true;
                break;
        }

        switch ((dma_ch[ch].ctrl.w >> 7) & 3) {
            case 0: src_inc =  unit_size; break;
            case 1: src_inc = -unit_size; break;
        }

        while (dma_count[ch]--) {
            if (dma_ch[ch].ctrl.w & DMA_32)
                arm_write(dma_dst_addr[ch],  arm_read(dma_src_addr[ch]));
            else
                arm_writeh(dma_dst_addr[ch], arm_readh(dma_src_addr[ch]));

            dma_dst_addr[ch] += dst_inc;
            dma_src_addr[ch] += src_inc;
        }

        if (dma_ch[ch].ctrl.w & DMA_IRQ)
            trigger_irq(DMA0_FLAG << ch);

        if (dma_ch[ch].ctrl.w & DMA_REP) {
            dma_count[ch] = dma_ch[ch].count.w;

            if (dst_reload) {
                dma_dst_addr[ch] = dma_ch[ch].dst.w;
            }

            continue;
        }

        dma_ch[ch].ctrl.w &= ~DMA_ENB;
    }
}

// Sound DMA-FIFO transfer was the cart->APU pipe for digital audio.
// With audio permanently disabled, the function is a no-op kept around
// only so existing callers in this TU still link. timer.c no longer
// invokes it either, so in practice it's never called.
void dma_transfer_gba_fifo(uint8_t ch) {
    (void)ch;
}