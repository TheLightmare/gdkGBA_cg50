#ifndef IO_H
#define IO_H

#include <stdbool.h>

typedef union {
    uint32_t w;

    struct {
        uint8_t b0;
        uint8_t b1;
        uint8_t b2;
        uint8_t b3;
    } b;
} io_reg;

#define VBLK_FLAG  (1 <<  0)
#define HBLK_FLAG  (1 <<  1)
#define VCNT_FLAG  (1 <<  2)
#define TMR0_FLAG  (1 <<  3)
#define TMR1_FLAG  (1 <<  4)
#define TMR2_FLAG  (1 <<  5)
#define TMR3_FLAG  (1 <<  6)
#define DMA0_FLAG  (1 <<  8)
#define DMA1_FLAG  (1 <<  9)
#define DMA2_FLAG  (1 << 10)
#define DMA3_FLAG  (1 << 11)

#define MAP_1D_FLAG  (1 <<  6)
#define BG0_ENB      (1 <<  8)
#define BG1_ENB      (1 <<  9)
#define BG2_ENB      (1 << 10)
#define BG3_ENB      (1 << 11)
#define OBJ_ENB      (1 << 12)

#define VBLK_IRQ  (1 <<  3)
#define HBLK_IRQ  (1 <<  4)
#define VCNT_IRQ  (1 <<  5)

extern io_reg disp_cnt;
extern io_reg green_inv;
extern io_reg disp_stat;
extern io_reg v_count;

typedef struct {
    io_reg ctrl;
    io_reg xofs;
    io_reg yofs;
} bg_t;

extern bg_t bg[4];

extern io_reg bg_pa[4];
extern io_reg bg_pb[4];
extern io_reg bg_pc[4];
extern io_reg bg_pd[4];

extern io_reg bg_refxe[4];
extern io_reg bg_refye[4];

extern io_reg bg_refxi[4];
extern io_reg bg_refyi[4];

extern io_reg win_in;
extern io_reg win_out;

extern io_reg bld_cnt;
extern io_reg bld_alpha;
extern io_reg bld_bright;

#define SWEEP_DEC  (1 <<  3)
#define ENV_INC    (1 << 11)
#define CH_LEN     (1 << 14)
#define WAVE_64    (1 <<  5)
#define WAVE_PLAY  (1 <<  7)
#define NOISE_7    (1 <<  3)

typedef struct {
    io_reg sweep;
    io_reg tone;
    io_reg ctrl;
} snd_sqr_ch_t;

typedef struct {
    io_reg wave;
    io_reg volume;
    io_reg ctrl;
} snd_wave_ch_t;

typedef struct {
    io_reg env;
    io_reg ctrl;
} snd_noise_ch_t;

extern snd_sqr_ch_t   sqr_ch[2];
extern snd_wave_ch_t  wave_ch;
extern snd_noise_ch_t noise_ch;

#define CH_SQR1_R   (1 <<  8)
#define CH_SQR2_R   (1 <<  9)
#define CH_WAVE_R   (1 << 10)
#define CH_NOISE_R  (1 << 11)
#define CH_SQR1_L   (1 << 12)
#define CH_SQR2_L   (1 << 13)
#define CH_WAVE_L   (1 << 14)
#define CH_NOISE_L  (1 << 15)
#define CH_DMAA_R   (1 <<  8)
#define CH_DMAA_L   (1 <<  9)
#define CH_DMAB_R   (1 << 12)
#define CH_DMAB_L   (1 << 13)
#define CH_SQR1     (1 <<  0)
#define CH_SQR2     (1 <<  1)
#define CH_WAVE     (1 <<  2)
#define CH_NOISE    (1 <<  3)
#define PSG_ENB     (1 <<  7)

extern io_reg snd_psg_vol;
extern io_reg snd_pcm_vol;
extern io_reg snd_psg_enb;
extern io_reg snd_bias;

extern uint8_t wave_ram[0x20];

extern int8_t snd_fifo_a_0;
extern int8_t snd_fifo_a_1;
extern int8_t snd_fifo_a_2;
extern int8_t snd_fifo_a_3;

extern int8_t snd_fifo_b_0;
extern int8_t snd_fifo_b_1;
extern int8_t snd_fifo_b_2;
extern int8_t snd_fifo_b_3;

typedef struct {
    io_reg src;
    io_reg dst;
    io_reg count;
    io_reg ctrl;
} dma_ch_t;

extern dma_ch_t dma_ch[4];

#define BTN_A    (1 << 0)
#define BTN_B    (1 << 1)
#define BTN_SEL  (1 << 2)
#define BTN_STA  (1 << 3)
#define BTN_R    (1 << 4)
#define BTN_L    (1 << 5)
#define BTN_U    (1 << 6)
#define BTN_D    (1 << 7)
#define BTN_RT   (1 << 8)
#define BTN_LT   (1 << 9)

typedef struct {
    io_reg count;
    io_reg reload;
    io_reg ctrl;
} tmr_t;

extern tmr_t tmr[4];

extern io_reg r_cnt;
extern io_reg sio_cnt;
extern io_reg sio_data8;
extern io_reg sio_data32;

extern io_reg key_input;

extern io_reg int_enb;
extern io_reg int_ack;
extern io_reg wait_cnt;
extern io_reg int_enb_m;

extern uint8_t ws_n[4];
extern uint8_t ws_s[4];

extern uint8_t ws_n_arm[4];
extern uint8_t ws_s_arm[4];

extern uint8_t ws_n_t16[4];
extern uint8_t ws_s_t16[4];

extern uint8_t post_boot;

extern bool io_open_bus;

uint8_t io_read(uint32_t address);

void io_write(uint32_t address, uint8_t value);

void trigger_irq(uint16_t flag);

void update_ws();

#endif