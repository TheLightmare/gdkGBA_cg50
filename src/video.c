#include "arm.h"
#include "arm_mem.h"

#include "dma.h"
#include "io.h"

#include <gint/display.h>
#include "gint_gba.h"

#include "sound.h"

// CG50 screen is 396x224 RGB565; gint_vram is its backing buffer.
// Center the 240x160 GBA viewport inside it.
#define CG_SCREEN_W   396
#define GBA_X_OFFSET  ((396 - 240) / 2)  // 78
#define GBA_Y_OFFSET  ((224 - 160) / 2)  // 32
#define SCREEN_PITCH  (CG_SCREEN_W * 2)
#define LINE_BYTE_OFFSET(line) \
    (((line) + GBA_Y_OFFSET) * SCREEN_PITCH + GBA_X_OFFSET * 2)

#define LINES_VISIBLE  160
#define LINES_TOTAL    228

#define CYC_LINE_TOTAL  1232
#define CYC_LINE_HBLK0  1006
#define CYC_LINE_HBLK1  (CYC_LINE_TOTAL - CYC_LINE_HBLK0)

void *screen;

static const uint8_t x_tiles_lut[16] = { 1, 2, 4, 8, 2, 4, 4, 8, 1, 1, 2, 4, 0, 0, 0, 0 };
static const uint8_t y_tiles_lut[16] = { 1, 2, 4, 8, 1, 1, 2, 4, 2, 4, 4, 8, 0, 0, 0, 0 };

static void render_obj(uint8_t prio) {
    if (!(disp_cnt.w & OBJ_ENB)) return;

    uint8_t obj_index;
    uint32_t offset = 0x3f8;

    uint32_t surf_addr = LINE_BYTE_OFFSET(v_count.w);

    for (obj_index = 0; obj_index < 128; obj_index++) {
        uint16_t attr0 = oam[offset + 0] | (oam[offset + 1] << 8);
        uint16_t attr1 = oam[offset + 2] | (oam[offset + 3] << 8);
        uint16_t attr2 = oam[offset + 4] | (oam[offset + 5] << 8);

        offset -= 8;

        int16_t  obj_y    = (attr0 >>  0) & 0xff;
        bool     affine   = (attr0 >>  8) & 0x1;
        bool     dbl_size = (attr0 >>  9) & 0x1;
        bool     hidden   = (attr0 >>  9) & 0x1;
        uint8_t  obj_shp  = (attr0 >> 14) & 0x3;
        uint8_t  affine_p = (attr1 >>  9) & 0x1f;
        uint8_t  obj_size = (attr1 >> 14) & 0x3;
        uint8_t  chr_prio = (attr2 >> 10) & 0x3;

        if (chr_prio != prio || (!affine && hidden)) continue;

        int16_t pa, pb, pc, pd;

        pa = pd = 0x100; //1.0
        pb = pc = 0x000; //0.0

        if (affine) {
            uint32_t p_base = affine_p * 32;

            pa = oam[p_base + 0x06] | (oam[p_base + 0x07] << 8);
            pb = oam[p_base + 0x0e] | (oam[p_base + 0x0f] << 8);
            pc = oam[p_base + 0x16] | (oam[p_base + 0x17] << 8);
            pd = oam[p_base + 0x1e] | (oam[p_base + 0x1f] << 8);
        }

        uint8_t lut_idx = obj_size | (obj_shp << 2);

        uint8_t x_tiles = x_tiles_lut[lut_idx];
        uint8_t y_tiles = y_tiles_lut[lut_idx];

        int32_t rcx = x_tiles * 4;
        int32_t rcy = y_tiles * 4;

        if (affine && dbl_size) {
            rcx *= 2;
            rcy *= 2;
        }

        if (obj_y + rcy * 2 > 0xff) obj_y -= 0x100;

        if (obj_y <= (int32_t)v_count.w && (obj_y + rcy * 2 > v_count.w)) {
            uint8_t  obj_mode = (attr0 >> 10) & 0x3;
            bool     mosaic   = (attr0 >> 12) & 0x1;
            bool     is_256   = (attr0 >> 13) & 0x1;
            int16_t  obj_x    = (attr1 >>  0) & 0x1ff;
            bool     flip_x   = (attr1 >> 12) & 0x1;
            bool     flip_y   = (attr1 >> 13) & 0x1;
            uint16_t chr_numb = (attr2 >>  0) & 0x3ff;
            uint8_t  chr_pal  = (attr2 >> 12) & 0xf;

            uint32_t chr_base = 0x10000 | chr_numb * 32;

            obj_x <<= 7;
            obj_x >>= 7;

            int32_t x, y = v_count.w - obj_y;

            if (!affine && flip_y) y ^= (y_tiles * 8) - 1;

            uint8_t tsz = is_256 ? 64 : 32; //Tile block size (in bytes, = (8 * 8 * bpp) / 8)
            uint8_t lsz = is_256 ?  8 :  4; //Pixel line row size (in bytes)

            int32_t ox = pa * -rcx + pb * (y - rcy) + (x_tiles << 10);
            int32_t oy = pc * -rcx + pd * (y - rcy) + (y_tiles << 10);

            if (!affine && flip_x) {
                ox = (x_tiles * 8 - 1) << 8;
                pa = -0x100;
            }

            uint32_t tys = (disp_cnt.w & MAP_1D_FLAG) ? x_tiles * tsz : 1024; //Tile row stride

            uint32_t address = surf_addr + obj_x * 2;

            for (x = 0; x < rcx * 2;
                x++,
                ox += pa,
                oy += pc,
                address += 2) {
                if (obj_x + x < 0) continue;
                if (obj_x + x >= 240) break;

                uint32_t vram_addr;
                uint32_t pal_idx;

                uint16_t tile_x = ox >> 11;
                uint16_t tile_y = oy >> 11;

                if (ox < 0 || tile_x >= x_tiles) continue;
                if (oy < 0 || tile_y >= y_tiles) continue;

                uint16_t chr_x = (ox >> 8) & 7;
                uint16_t chr_y = (oy >> 8) & 7;

                uint32_t chr_addr =
                    chr_base       +
                    tile_y   * tys +
                    chr_y    * lsz;

                if (is_256) {
                    vram_addr = chr_addr + tile_x * 64 + chr_x;
                    pal_idx   = vram[vram_addr];
                } else {
                    vram_addr = chr_addr + tile_x * 32 + (chr_x >> 1);
                    pal_idx   = (vram[vram_addr] >> (chr_x & 1) * 4) & 0xf;
                }

                uint32_t pal_addr = 0x100 | pal_idx | (!is_256 ? chr_pal * 16 : 0);

                if (pal_idx) *(uint16_t *)((uint8_t *)screen + address) = palette[pal_addr];
            }
        }
    }
}

static const uint8_t bg_enb[3] = { 0xf, 0x7, 0xc };

static void render_bg() {
    uint8_t mode = disp_cnt.w & 7;

    uint32_t surf_addr = LINE_BYTE_OFFSET(v_count.w);

    switch (mode) {
        case 0:
        case 1:
        case 2: {
            uint8_t enb = (disp_cnt.w >> 8) & bg_enb[mode];

            int8_t prio, bg_idx;

            for (prio = 3; prio >= 0; prio--) {
                for (bg_idx = 3; bg_idx >= 0; bg_idx--) {
                    if (!(enb & (1 << bg_idx))) continue;
                    if ((bg[bg_idx].ctrl.w & 3) != prio) continue;

                    uint32_t chr_base  = ((bg[bg_idx].ctrl.w >>  2) & 0x3)  << 14;
                    bool     is_256    =  (bg[bg_idx].ctrl.w >>  7) & 0x1;
                    uint16_t scrn_base = ((bg[bg_idx].ctrl.w >>  8) & 0x1f) << 11;
                    bool     aff_wrap  =  (bg[bg_idx].ctrl.w >> 13) & 0x1;
                    uint16_t scrn_size =  (bg[bg_idx].ctrl.w >> 14);

                    bool affine = mode == 2 || (mode == 1 && bg_idx == 2);

                    uint32_t address = surf_addr;

                    if (affine) {
                        int16_t pa = bg_pa[bg_idx].w;
                        int16_t pb = bg_pb[bg_idx].w;
                        int16_t pc = bg_pc[bg_idx].w;
                        int16_t pd = bg_pd[bg_idx].w;

                        int32_t ox = ((int32_t)bg_refxi[bg_idx].w << 4) >> 4;
                        int32_t oy = ((int32_t)bg_refyi[bg_idx].w << 4) >> 4;

                        bg_refxi[bg_idx].w += pb;
                        bg_refyi[bg_idx].w += pd;

                        uint8_t tms = 16 << scrn_size;
                        uint8_t tmsk = tms - 1;

                        uint8_t x;

                        for (x = 0; x < 240;
                            x++,
                            ox += pa,
                            oy += pc,
                            address += 2) {
                            int16_t tmx = ox >> 11;
                            int16_t tmy = oy >> 11;

                            if (aff_wrap) {
                                tmx &= tmsk;
                                tmy &= tmsk;
                            } else {
                                if (tmx < 0 || tmx >= tms) continue;
                                if (tmy < 0 || tmy >= tms) continue;
                            }

                            uint16_t chr_x = (ox >> 8) & 7;
                            uint16_t chr_y = (oy >> 8) & 7;

                            uint32_t map_addr = scrn_base + tmy * tms + tmx;

                            uint32_t vram_addr = chr_base + vram[map_addr] * 64 + chr_y * 8 + chr_x;

                            uint16_t pal_idx = vram[vram_addr];

                            if (pal_idx) *(uint16_t *)((uint8_t *)screen + address) = palette[pal_idx];
                        }
                    } else {
                        uint16_t oy     = v_count.w + bg[bg_idx].yofs.w;
                        uint16_t tmy    = oy >> 3;
                        uint16_t scrn_y = (tmy >> 5) & 1;

                        uint8_t x;

                        for (x = 0; x < 240; x++) {
                            uint16_t ox     = x + bg[bg_idx].xofs.w;
                            uint16_t tmx    = ox >> 3;
                            uint16_t scrn_x = (tmx >> 5) & 1;

                            uint16_t chr_x = ox & 7;
                            uint16_t chr_y = oy & 7;

                            uint16_t pal_idx;
                            uint16_t pal_base = 0;

                            uint32_t map_addr = scrn_base + (tmy & 0x1f) * 32 * 2 + (tmx & 0x1f) * 2;

                            switch (scrn_size) {
                                case 1: map_addr += scrn_x * 2048; break;
                                case 2: map_addr += scrn_y * 2048; break;
                                case 3: map_addr += scrn_x * 2048 + scrn_y * 4096; break;
                            }

                            uint16_t tile = vram[map_addr + 0] | (vram[map_addr + 1] << 8);

                            uint16_t chr_numb = (tile >>  0) & 0x3ff;
                            bool     flip_x   = (tile >> 10) & 0x1;
                            bool     flip_y   = (tile >> 11) & 0x1;
                            uint8_t  chr_pal  = (tile >> 12) & 0xf;

                            if (!is_256) pal_base = chr_pal * 16;

                            if (flip_x) chr_x ^= 7;
                            if (flip_y) chr_y ^= 7;

                            uint32_t vram_addr;

                            if (is_256) {
                                vram_addr = chr_base + chr_numb * 64 + chr_y * 8 + chr_x;
                                pal_idx   = vram[vram_addr];
                            } else {
                                vram_addr = chr_base + chr_numb * 32 + chr_y * 4 + (chr_x >> 1);
                                pal_idx   = (vram[vram_addr] >> (chr_x & 1) * 4) & 0xf;
                            }

                            uint32_t pal_addr = pal_idx | pal_base;

                            if (pal_idx) *(uint16_t *)((uint8_t *)screen + address) = palette[pal_addr];

                            address += 2;
                        }
                    }
                }

                render_obj(prio);
            }
        }
        break;

        case 3: {
            uint8_t x;
            uint32_t frm_addr = v_count.w * 480;

            for (x = 0; x < 240; x++) {
                uint16_t pixel = vram[frm_addr + 0] | (vram[frm_addr + 1] << 8);

                uint16_t r = (pixel >>  0) & 0x1f;
                uint16_t g = (pixel >>  5) & 0x1f;
                uint16_t b = (pixel >> 10) & 0x1f;

                *(uint16_t *)((uint8_t *)screen + surf_addr) =
                    (r << 11) | (((g << 1) | (g >> 4)) << 5) | b;

                surf_addr += 2;

                frm_addr += 2;
            }
        }
        break;

        case 4: {
            uint8_t x, frame = (disp_cnt.w >> 4) & 1;
            uint32_t frm_addr = 0xa000 * frame + v_count.w * 240;

            for (x = 0; x < 240; x++) {
                uint8_t pal_idx = vram[frm_addr++];

                *(uint16_t *)((uint8_t *)screen + surf_addr) = palette[pal_idx];

                surf_addr += 2;
            }
        }
        break;
    }
}

static void render_line() {
    uint16_t *line = (uint16_t *)((uint8_t *)screen + LINE_BYTE_OFFSET(v_count.w));

    // Forced-blank: real hardware drives a white screen and disables all
    // rendering. Match that so the user sees something during BIOS boot
    // instead of a black rectangle.
    if (disp_cnt.w & FORCED_BLANK) {
        for (int i = 0; i < 240; i++) line[i] = 0xffff;
        return;
    }

    uint16_t backdrop = palette[0];
    for (int i = 0; i < 240; i++) line[i] = backdrop;

    if ((disp_cnt.w & 7) > 2) {
        render_bg(0);
        render_obj(0);
        render_obj(1);
        render_obj(2);
        render_obj(3);
    } else {
        render_bg();
    }
}

static void vblank_start() {
    if (disp_stat.w & VBLK_IRQ) trigger_irq(VBLK_FLAG);

    disp_stat.w |= VBLK_FLAG;
}

static void hblank_start() {
    if (disp_stat.w & HBLK_IRQ) trigger_irq(HBLK_FLAG);

    disp_stat.w |= HBLK_FLAG;
}

static void vcount_match() {
    if (disp_stat.w & VCNT_IRQ) trigger_irq(VCNT_FLAG);

    disp_stat.w |= VCNT_FLAG;
}

// Frame skip: render + dupdate only 1 in every (FRAMESKIP+1) frames. The
// cart's main loop is unaffected -- arm_exec, IRQs, DMAs, and v_count all
// advance at full rate -- so game logic keeps real-time. We just elide the
// per-pixel render_line() work and the ~11 ms dupdate() flush on skipped
// frames. Display ends up showing every (FRAMESKIP+1)-th frame, which the
// human eye accepts much more readily than emulator slowdown.
//   FRAMESKIP=0: render every frame  (no skipping)
//   FRAMESKIP=1: render every other  (recommended)
//   FRAMESKIP=2: render 1 in 3       (very fast, choppier)
#ifndef FRAMESKIP
#define FRAMESKIP 1
#endif

void run_frame() {
    static uint32_t skip_phase = 0;
    bool render_this_frame = (skip_phase++ % (FRAMESKIP + 1)) == 0;

    disp_stat.w &= ~VBLK_FLAG;

    screen = gint_vram;

    for (v_count.w = 0; v_count.w < LINES_TOTAL; v_count.w++) {
        disp_stat.w &= ~(HBLK_FLAG | VCNT_FLAG);

        //V-Count match and V-Blank start
        if (v_count.w == disp_stat.b.b1) vcount_match();

        if (v_count.w == LINES_VISIBLE) {
            bg_refxi[2].w = bg_refxe[2].w;
            bg_refyi[2].w = bg_refye[2].w;

            bg_refxi[3].w = bg_refxe[3].w;
            bg_refyi[3].w = bg_refye[3].w;

            vblank_start();
            dma_transfer_gba(VBLANK);
        }

        arm_exec(CYC_LINE_HBLK0);

        //H-Blank start
        if (v_count.w < LINES_VISIBLE) {
            if (render_this_frame) render_line();
            dma_transfer_gba(HBLANK);
        }

        hblank_start();

        arm_exec(CYC_LINE_HBLK1);

        // Sound is not piped to any output yet, and sound_clock() does
        // double-precision FP math per scanline that is very slow on the
        // SH4 (no hardware FPU usage by gint). Skip it for now.
        // sound_clock(CYC_LINE_TOTAL);
    }

    // If we're skipping the render+dupdate this frame, return now. The
    // diagnostic overlay below would otherwise update gint_vram and trigger
    // the dupdate that we're trying to avoid.
    if (!render_this_frame) return;

    // Frame heartbeat + diagnostic overlay. Lives in the white margin below
    // the centered GBA viewport so it never collides with the emulated frame.
    {
        extern arm_regs_t arm_r;
        extern bool int_halt;
        extern io_reg int_enb, int_ack, int_enb_m;

        static uint32_t frame_count = 0;
        static const uint16_t hb_colors[4] = { 0xf800, 0x07e0, 0x001f, 0xffff };
        uint16_t c = hb_colors[frame_count & 3];
        for (int y = 4; y < 14; y++)
            for (int x = 380; x < 392; x++)
                gint_vram[y * CG_SCREEN_W + x] = c;

        // Clear the status strip (rows 196-223) so text doesn't accumulate.
        for (int y = 196; y < 224; y++)
            for (int x = 0; x < CG_SCREEN_W; x++)
                gint_vram[y * CG_SCREEN_W + x] = 0xffff;

        // Probe up to 4 raw bytes at PC so we can read the loop instruction.
        uint32_t pc = arm_r.r[15];
        uint8_t b0 = arm_readb(pc + 0);
        uint8_t b1 = arm_readb(pc + 1);
        uint8_t b2 = arm_readb(pc + 2);
        uint8_t b3 = arm_readb(pc + 3);

        dprint(2, 196, C_BLACK,
               "F:%lu PC:%08lX op:%02X%02X%02X%02X DISPCNT:%04X PAL0:%04X",
               (unsigned long)frame_count,
               (unsigned long)pc,
               b0, b1, b2, b3,
               (unsigned)disp_cnt.w,
               (unsigned)palette[0]);

        dprint(2, 210, C_BLACK,
               "CPSR:%08lX halt:%d IME:%X IE:%04X IF:%04X",
               (unsigned long)arm_r.cpsr,
               (int)int_halt,
               (unsigned)(int_enb_m.w & 1),
               (unsigned)int_enb.w,
               (unsigned)int_ack.w);

        frame_count++;
    }

    // gint_vram is the active framebuffer; just push it to the display.
    dupdate();

    // sound_buffer_wrap();   // see note above on sound_clock()
}