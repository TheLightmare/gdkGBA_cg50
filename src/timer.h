#ifndef TIMER_H
#define TIMER_H

#define TMR_CASCADE  (1 << 2)
#define TMR_IRQ      (1 << 6)
#define TMR_ENB      (1 << 7)

extern uint32_t tmr_icnt[4];

extern uint8_t tmr_enb;
extern uint8_t tmr_irq;
extern uint8_t tmr_ie;

void timers_clock(uint32_t cycles);

#endif