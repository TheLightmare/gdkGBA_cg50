// gdkGBA's sound subsystem has been permanently disabled in this port.
//
// Reasons:
// 1. There is no audio output backend wired up to gint.
// 2. The original sound_clock() / square_sample() / etc. relied heavily on
//    double-precision floating-point arithmetic. The SH4 in the CG50 has
//    no FPU exposed through gint, so doubles are software-emulated -- a
//    crippling per-scanline cost that swamped real emulation work.
// 3. The cart-side timer/DMA paths that fed the FIFOs were running every
//    sound-rate timer overflow (~32 kHz) and have been removed from
//    timer.c and dma.c.
//
// What remains in this TU is only the global state that io.c reads from
// and writes to (sound IO registers and FIFO bytes). io.c just stores
// values into these globals; nothing here ever reads them back to
// produce audio. The state is kept so that a future audio implementation
// can be reattached without re-wiring io.c.

#include <stdint.h>

#include "io.h"
#include "sound.h"

// FIFO buffers + lengths. Written by io.c FIFO_A/B register stores; read
// previously by fifo_a_load/fifo_b_load (now stubbed). Kept to preserve
// the io.c API surface.
int8_t fifo_a[0x20];
int8_t fifo_b[0x20];

uint8_t fifo_a_len;
uint8_t fifo_b_len;

// Per-channel synthesis state. Now never updated, but io_read returns of
// some sound registers reference snd_ch_state via macros, and the typedef
// is exposed in sound.h.
snd_ch_state_t snd_ch_state[4];

uint8_t wave_position;
uint8_t wave_samples;

// All the sound-API entry points are stubs. They exist so external code
// still links cleanly. video.c's run_frame already has the
// sound_clock/sound_buffer_wrap calls commented out; timer.c and dma.c
// have had their fifo_*_load / fifo_*_copy calls removed entirely.
void wave_reset(void)            {}
void sound_buffer_wrap(void)     {}
void sound_mix(void *data, uint8_t *stream, int32_t len) { (void)data; (void)stream; (void)len; }
void sound_clock(uint32_t cycles){ (void)cycles; }
void fifo_a_copy(void)           {}
void fifo_b_copy(void)           {}
void fifo_a_load(void)           {}
void fifo_b_load(void)           {}
