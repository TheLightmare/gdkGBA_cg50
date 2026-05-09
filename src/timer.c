#include "arm.h"

#include "dma.h"
#include "io.h"
#include "timer.h"

//EXTERN VARIABLES DECLARATION ===

uint32_t tmr_icnt[4];

uint8_t tmr_enb;
uint8_t tmr_irq;
uint8_t tmr_ie;

//================================

static const uint8_t pscale_shift_lut[4]  = { 0, 6, 8, 10 };

void timers_clock(uint32_t cycles) {
    uint8_t idx;
    bool overflow = false;

    for (idx = 0; idx < 4; idx++) {
        if (!(tmr[idx].ctrl.w & TMR_ENB)) {
            overflow = false;
            continue;
        }

        if (tmr[idx].ctrl.w & TMR_CASCADE) {
            if (overflow) tmr[idx].count.w++;
        } else {
            uint8_t shift = pscale_shift_lut[tmr[idx].ctrl.w & 3];
            uint32_t inc = (tmr_icnt[idx] += cycles) >> shift;

            tmr[idx].count.w += inc;
            tmr_icnt[idx] -= inc << shift;
        }

        if ((overflow = (tmr[idx].count.w > 0xffff))) {
            tmr[idx].count.w = tmr[idx].reload.w + (tmr[idx].count.w - 0x10000);

            // Sound DMA-FIFO triggers removed: this emulator has no audio
            // backend and the per-timer-overflow fifo_a_load/fifo_b_load
            // calls were a measurable cost (sound timers run at sample
            // rates, ~32 kHz, multiplied by emulated frames). Audio is
            // permanently disabled.
        }

        if ((tmr[idx].ctrl.w & TMR_IRQ) && overflow)
            trigger_irq(TMR0_FLAG << idx);
    }
}