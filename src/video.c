#include "arm.h"
#include "arm_mem.h"
#include "bench.h"
#include "build_flags.h"

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

// Per-pixel BG/OBJ enable mask for the current scanline. Only populated
// when at least one window is enabled. The render fast path skips the
// per-pixel mask check entirely when windows_active is false, avoiding
// layer_mask memory traffic on the (very common) games-without-windows
// path.
//   bit 0 = BG0 visible
//   bit 1 = BG1 visible
//   bit 2 = BG2 visible
//   bit 3 = BG3 visible
//   bit 4 = OBJ visible
//
// The OBJ window (DISPCNT bit 15) is not yet implemented — it would need
// a pre-render pass to mark sprite-touching pixels — so games using only
// it will still render their OBJs everywhere.
static uint8_t layer_mask[240];
static bool    windows_active;

// Per-pixel layer-ID record for the current scanline. Tracks which layer
// drew the topmost pixel so blending can decide whether the layer
// underneath is a "second target" for BLDCNT mode 1, and so OBJ
// semi-transparent (obj_mode=1) sprites can blend with whatever's
// underneath regardless of BLDCNT.
//
// Values: 0..3 = BG0..BG3, 4 = OBJ, 5 = backdrop.
//
// Updated unconditionally on every pixel write (one byte per pixel) so
// the blend code at the next layer can read it back. The cost is small
// vs. the existing per-pixel work.
#define LAYER_BG0  0
#define LAYER_BG1  1
#define LAYER_BG2  2
#define LAYER_BG3  3
#define LAYER_OBJ  4
#define LAYER_BD   5
#define LAYER_BIT(L)  (1u << (L))
static uint8_t layer_id_buf[240];

// Blend state, refreshed at the start of each scanline from BLDCNT /
// BLDALPHA / BLDY. Cached in scanline-local statics so the per-pixel
// blend path doesn't re-read the IO regs and re-mask each time.
//
//   blend_mode  -- BLDCNT bits 6-7: 0 none, 1 alpha, 2 brightness inc,
//                  3 brightness dec
//   blend_first -- BLDCNT bits 0-5: 1st-target layer mask
//                  (bit 0=BG0..bit4=OBJ, bit5=backdrop)
//   blend_second-- BLDCNT bits 8-13: 2nd-target layer mask
//   blend_eva   -- BLDALPHA bits 0-4 clamped to [0,16]
//   blend_evb   -- BLDALPHA bits 8-12 clamped to [0,16]
//   blend_evy   -- BLDY bits 0-4 clamped to [0,16]
//   blend_any   -- true if BLDCNT mode != 0 OR any sprite this line could
//                  be semi-trans. We don't pre-scan OAM for the latter,
//                  so just set it true whenever OBJ is enabled in
//                  DISPCNT -- the per-pixel OBJ writer will short-
//                  circuit on obj_mode != 1 anyway.
static uint8_t blend_mode;
static uint8_t blend_first;
static uint8_t blend_second;
static uint8_t blend_eva;
static uint8_t blend_evb;
static uint8_t blend_evy;
static bool    blend_any;

static void compute_blend_state(void) {
    blend_mode   = (bld_cnt.w >> 6) & 3;
    blend_first  = bld_cnt.b.b0 & 0x3f;
    blend_second = bld_cnt.b.b1 & 0x3f;
    uint8_t eva = bld_alpha.b.b0 & 0x1f;
    blend_eva = (eva > 16) ? 16 : eva;
    uint8_t evb = bld_alpha.b.b1 & 0x1f;
    blend_evb = (evb > 16) ? 16 : evb;
    uint8_t evy = bld_bright.b.b0 & 0x1f;
    blend_evy = (evy > 16) ? 16 : evy;

    // Skip the per-pixel layer-ID buffer maintenance entirely when
    // nothing this scanline could need it. OBJ semi-trans is always
    // possible if OBJ is enabled, so include that case.
    blend_any = (blend_mode != 0) || (disp_cnt.w & OBJ_ENB);
}

// RGB565 alpha blend: result = clamp((src * eva + dst * evb) / 16) per
// channel. eva and evb are 0..16.
static inline uint16_t blend_alpha_rgb565(uint16_t src, uint16_t dst,
                                          uint8_t eva, uint8_t evb) {
    uint32_t sr = (src >> 11) & 0x1f;
    uint32_t sg = (src >>  5) & 0x3f;
    uint32_t sb =  src        & 0x1f;
    uint32_t dr = (dst >> 11) & 0x1f;
    uint32_t dg = (dst >>  5) & 0x3f;
    uint32_t db =  dst        & 0x1f;
    uint32_t r = (sr * eva + dr * evb) >> 4;
    uint32_t g = (sg * eva + dg * evb) >> 4;
    uint32_t b = (sb * eva + db * evb) >> 4;
    if (r > 31) r = 31;
    if (g > 63) g = 63;
    if (b > 31) b = 31;
    return (uint16_t)((r << 11) | (g << 5) | b);
}

// Apply blend mode to a freshly-drawn `src` pixel from layer `src_layer`,
// given the existing pixel `dst` from layer `dst_layer`. Returns the
// pixel to actually write. When blending doesn't apply, returns `src`
// unchanged.
//
// Special-case OBJ semi-transparent: an OBJ pixel with attr0 mode == 1
// alpha-blends with the underlying pixel regardless of BLDCNT mode, as
// long as the underlying is in the second-target mask. The caller
// passes obj_semitrans=true for that case.
static inline uint16_t blend_apply(uint16_t src, uint8_t src_layer,
                                   uint16_t dst, uint8_t dst_layer,
                                   bool obj_semitrans) {
    if (obj_semitrans) {
        // OBJ semi-trans is alpha blend with EVA/EVB, gated only by the
        // 2nd-target check; effectively overrides the BLDCNT mode.
        if (blend_second & LAYER_BIT(dst_layer)) {
            return blend_alpha_rgb565(src, dst, blend_eva, blend_evb);
        }
        return src;
    }

    if (blend_mode == 0) return src;
    if (!(blend_first & LAYER_BIT(src_layer))) return src;

    if (blend_mode == 1) {
        // Alpha: only when the layer underneath is a 2nd target.
        if (!(blend_second & LAYER_BIT(dst_layer))) return src;
        return blend_alpha_rgb565(src, dst, blend_eva, blend_evb);
    }
    // Brightness modes don't read `dst`.
    uint32_t r = (src >> 11) & 0x1f;
    uint32_t g = (src >>  5) & 0x3f;
    uint32_t b =  src        & 0x1f;
    if (blend_mode == 2) {
        // inc: c += (max - c) * EVY / 16
        r += ((31 - r) * blend_evy) >> 4;
        g += ((63 - g) * blend_evy) >> 4;
        b += ((31 - b) * blend_evy) >> 4;
    } else {
        // mode == 3, dec: c -= c * EVY / 16
        r -= (r * blend_evy) >> 4;
        g -= (g * blend_evy) >> 4;
        b -= (b * blend_evy) >> 4;
    }
    return (uint16_t)((r << 11) | (g << 5) | b);
}

// Per-pixel write helpers used by the BG and OBJ rasterisers. They keep
// layer_id_buf consistent and apply BLDCNT when applicable. The
// blend_any short-circuit means the only added cost on a no-blend
// scanline is one byte store per pixel.
static inline void write_bg_pixel(uint8_t *screen_b, uint32_t addr,
                                  uint16_t color, uint8_t pixel_x,
                                  uint8_t bg_layer) {
    uint16_t *p = (uint16_t *)(screen_b + addr);
    if (blend_any) {
        if (blend_mode != 0 && (blend_first & LAYER_BIT(bg_layer))) {
            uint16_t dst = *p;
            uint8_t  dst_layer = layer_id_buf[pixel_x];
            color = blend_apply(color, bg_layer, dst, dst_layer, false);
        }
        layer_id_buf[pixel_x] = bg_layer;
    }
    *p = color;
}

static inline void write_obj_pixel(uint8_t *screen_b, uint32_t addr,
                                   uint16_t color, uint8_t pixel_x,
                                   uint8_t obj_mode) {
    uint16_t *p = (uint16_t *)(screen_b + addr);
    bool semitrans = (obj_mode == 1);
    if (blend_any) {
        if (semitrans ||
            (blend_mode != 0 && (blend_first & LAYER_BIT(LAYER_OBJ)))) {
            uint16_t dst = *p;
            uint8_t  dst_layer = layer_id_buf[pixel_x];
            color = blend_apply(color, LAYER_OBJ, dst, dst_layer, semitrans);
        }
        layer_id_buf[pixel_x] = LAYER_OBJ;
    }
    *p = color;
}

static void compute_layer_mask(void) {
    bool win0_on   = (disp_cnt.w >> 13) & 1;
    bool win1_on   = (disp_cnt.w >> 14) & 1;
    bool objwin_on = (disp_cnt.w >> 15) & 1;

    windows_active = win0_on || win1_on || objwin_on;
    if (!windows_active) {
        // Fast path: no windows are active for this scanline. Skip the
        // memset; render code will short-circuit the mask check.
        return;
    }

    // Default for any pixel not covered by an active window.
    uint8_t outside_mask = win_out.b.b0 & 0x1f;
    for (int x = 0; x < 240; x++) layer_mask[x] = outside_mask;

    // WIN1 first (lower priority — WIN0 overrides if both cover a pixel).
    if (win1_on) {
        uint8_t y1 = win1_v.b.b1;       // top
        uint16_t y2 = win1_v.b.b0;      // bottom (exclusive)
        if (y2 > 160 || y1 > y2) y2 = 160;
        if (v_count.w >= y1 && v_count.w < y2) {
            uint8_t in_mask = win_in.b.b1 & 0x1f;
            uint8_t  x1 = win1_h.b.b1;
            uint16_t x2 = win1_h.b.b0;
            if (x2 > 240 || x1 > x2) x2 = 240;
            for (int x = x1; x < x2; x++) layer_mask[x] = in_mask;
        }
    }

    // WIN0 second (higher priority).
    if (win0_on) {
        uint8_t y1 = win0_v.b.b1;
        uint16_t y2 = win0_v.b.b0;
        if (y2 > 160 || y1 > y2) y2 = 160;
        if (v_count.w >= y1 && v_count.w < y2) {
            uint8_t in_mask = win_in.b.b0 & 0x1f;
            uint8_t  x1 = win0_h.b.b1;
            uint16_t x2 = win0_h.b.b0;
            if (x2 > 240 || x1 > x2) x2 = 240;
            for (int x = x1; x < x2; x++) layer_mask[x] = in_mask;
        }
    }
}

// Per-line sprite list precomputation
//
// render_obj used to walk all 128 OAM entries every time it was called
// (once per priority, four times per scanline = 81,920 entry checks per
// frame). Most of that work was repeated decoding -- read attr0/1/2,
// extract ~10 bit-fields, compute extents, do the visibility check --
// just to bail on entries that didn't apply to this priority or this
// scanline.
//
// The lists below are populated once per rendered frame from
// build_sprite_lists(). They map (scanline, priority) to the list of
// OAM indices visible there. render_obj just walks the matching list.
//
// Stacking order: GBA convention is "lower OAM index = drawn last (on
// top) within a priority". We walk OAM 0..127 forward when building so
// that on overflow the BACK sprites (high OAM) are dropped first --
// front sprites stay visible. The renderer iterates the list in reverse
// to get back-to-front compositing.
//
// Cap is 32 sprites per (line, priority). Real GBA has a per-scanline
// cycle budget for OBJ rendering rather than a count cap; 32 covers the
// vast majority of game scenes. Mid-frame OAM writes (rare; almost
// always done during VBlank or via VBlank-DMA) won't be reflected
// until the next frame.

#define OBJ_LINE_CAP  32

#define DECOBJ_FLAG_AFFINE  (1u << 0)
#define DECOBJ_FLAG_FLIPX   (1u << 1)
#define DECOBJ_FLAG_FLIPY   (1u << 2)
#define DECOBJ_FLAG_IS256   (1u << 3)
#define DECOBJ_FLAG_DROP    (1u << 4)  // non-affine hidden sprite

typedef struct {
    int16_t  obj_x, obj_y;            // signed; obj_y is post Y-wrap
    int16_t  rcx, rcy;                 // half-extent in pixels
    int16_t  pa, pb, pc, pd;           // affine matrix; (1.0,0,0,1.0) for non-affine
    // chr_base = 0x10000 | (chr_numb * 32); chr_numb is up to 0x3ff so the
    // value can reach 0x17FE0 -- needs 17 bits. Storing this in a
    // uint16_t silently truncates the 0x10000 base, sending sprites to
    // read from the BG tile region instead of the OBJ tile region.
    uint32_t chr_base;
    uint16_t tys;                      // tile-row stride bytes
    uint8_t  x_tiles, y_tiles;
    uint8_t  priority;                 // 0..3
    uint8_t  obj_mode;                 // 0 normal, 1 semi-trans, 2 obj-window
    uint8_t  chr_pal;
    uint8_t  flags;
} DecodedObj;

static DecodedObj decoded_objs[128];
static uint8_t    obj_line_idx[160][4][OBJ_LINE_CAP];
static uint8_t    obj_line_count[160][4];

static void decode_one_obj(uint8_t idx, DecodedObj *obj) {
    uint32_t off = (uint32_t)idx * 8u;
    uint16_t attr0 = oam[off + 0] | (oam[off + 1] << 8);
    uint16_t attr1 = oam[off + 2] | (oam[off + 3] << 8);
    uint16_t attr2 = oam[off + 4] | (oam[off + 5] << 8);

    bool affine     = (attr0 >>  8) & 0x1;
    bool dbl_size   = (attr0 >>  9) & 0x1;
    bool hidden     = (attr0 >>  9) & 0x1;
    uint8_t obj_shp = (attr0 >> 14) & 0x3;
    uint8_t affine_p= (attr1 >>  9) & 0x1f;
    uint8_t obj_size= (attr1 >> 14) & 0x3;

    int16_t obj_y = (attr0 >> 0) & 0xff;
    int16_t obj_x = (attr1 >> 0) & 0x1ff;
    obj_x <<= 7;
    obj_x >>= 7;

    bool is_256 = (attr0 >> 13) & 0x1;
    bool flip_x = (attr1 >> 12) & 0x1;
    bool flip_y = (attr1 >> 13) & 0x1;

    obj->priority = (attr2 >> 10) & 0x3;
    obj->obj_mode = (attr0 >> 10) & 0x3;
    obj->chr_pal  = (attr2 >> 12) & 0xf;
    obj->obj_x    = obj_x;

    uint16_t chr_numb = attr2 & 0x3ff;
    obj->chr_base = (uint32_t)0x10000u | ((uint32_t)chr_numb * 32u);

    uint8_t lut_idx = (uint8_t)(obj_size | (obj_shp << 2));
    uint8_t x_tiles = x_tiles_lut[lut_idx];
    uint8_t y_tiles = y_tiles_lut[lut_idx];
    obj->x_tiles = x_tiles;
    obj->y_tiles = y_tiles;

    int32_t rcx = x_tiles * 4;
    int32_t rcy = y_tiles * 4;
    if (affine && dbl_size) {
        rcx *= 2;
        rcy *= 2;
    }
    obj->rcx = (int16_t)rcx;
    obj->rcy = (int16_t)rcy;

    if (obj_y + rcy * 2 > 0xff) obj_y -= 0x100;
    obj->obj_y = obj_y;

    uint8_t tsz = is_256 ? 64 : 32;
    obj->tys = (uint16_t)((disp_cnt.w & MAP_1D_FLAG) ? x_tiles * tsz : 1024);

    uint8_t flags = 0;
    if (affine) flags |= DECOBJ_FLAG_AFFINE;
    if (flip_x) flags |= DECOBJ_FLAG_FLIPX;
    if (flip_y) flags |= DECOBJ_FLAG_FLIPY;
    if (is_256) flags |= DECOBJ_FLAG_IS256;
    if (!affine && hidden) flags |= DECOBJ_FLAG_DROP;
    obj->flags = flags;

    if (affine) {
        uint32_t p_base = (uint32_t)affine_p * 32u;
        obj->pa = (int16_t)(oam[p_base + 0x06] | (oam[p_base + 0x07] << 8));
        obj->pb = (int16_t)(oam[p_base + 0x0e] | (oam[p_base + 0x0f] << 8));
        obj->pc = (int16_t)(oam[p_base + 0x16] | (oam[p_base + 0x17] << 8));
        obj->pd = (int16_t)(oam[p_base + 0x1e] | (oam[p_base + 0x1f] << 8));
    } else {
        obj->pa = obj->pd = 0x100;
        obj->pb = obj->pc = 0x000;
    }
}

static void build_sprite_lists(void) {
    if (!(disp_cnt.w & OBJ_ENB)) return;

    // Reset counts. The lists themselves are written below; only the
    // portion we actually push to is read by the renderer.
    for (int y = 0; y < 160; y++) {
        obj_line_count[y][0] = 0;
        obj_line_count[y][1] = 0;
        obj_line_count[y][2] = 0;
        obj_line_count[y][3] = 0;
    }

    for (uint8_t idx = 0; idx < 128; idx++) {
        DecodedObj *obj = &decoded_objs[idx];
        decode_one_obj(idx, obj);

        if (obj->flags & DECOBJ_FLAG_DROP) continue;

        int y_start = obj->obj_y;
        int y_end   = obj->obj_y + obj->rcy * 2;
        if (y_end <= 0)     continue;
        if (y_start >= 160) continue;
        if (y_start < 0)    y_start = 0;
        if (y_end > 160)    y_end = 160;

        uint8_t prio = obj->priority;
        for (int y = y_start; y < y_end; y++) {
            uint8_t cnt = obj_line_count[y][prio];
            if (cnt < OBJ_LINE_CAP) {
                obj_line_idx[y][prio][cnt] = idx;
                obj_line_count[y][prio] = cnt + 1;
            }
        }
    }
}

// Render one already-decoded sprite at the current scanline.
// This is the per-sprite body that used to live inline in render_obj.
static void render_decoded_obj(const DecodedObj *obj) {
    bool affine = obj->flags & DECOBJ_FLAG_AFFINE;
    bool flip_x = obj->flags & DECOBJ_FLAG_FLIPX;
    bool flip_y = obj->flags & DECOBJ_FLAG_FLIPY;
    bool is_256 = obj->flags & DECOBJ_FLAG_IS256;

    uint32_t surf_addr = LINE_BYTE_OFFSET(v_count.w);

    int32_t y = (int32_t)v_count.w - obj->obj_y;
    if (!affine && flip_y) y ^= (obj->y_tiles * 8) - 1;

    int16_t pa = obj->pa;
    int16_t pb = obj->pb;
    int16_t pc = obj->pc;
    int16_t pd = obj->pd;

    int32_t rcx = obj->rcx;
    int32_t rcy = obj->rcy;
    int32_t x_tiles = obj->x_tiles;
    int32_t y_tiles = obj->y_tiles;
    int16_t obj_x   = obj->obj_x;

    int32_t ox = pa * -rcx + pb * (y - rcy) + (x_tiles << 10);
    int32_t oy = pc * -rcx + pd * (y - rcy) + (y_tiles << 10);

    if (!affine && flip_x) {
        ox = (x_tiles * 8 - 1) << 8;
        pa = -0x100;
    }

    uint32_t address = surf_addr + obj_x * 2;
    uint32_t chr_base = obj->chr_base;
    uint16_t tys      = obj->tys;
    uint8_t  obj_mode = obj->obj_mode;
    uint8_t  chr_pal  = obj->chr_pal;
    uint8_t  lsz = is_256 ? 8 : 4;

    if (!affine) {
        // Non-affine fast path: oy is constant across this scanline,
        // and ox steps linearly by +/-256 per pixel (pa = +0x100 or
        // -0x100 for flip_x). tile_y, chr_y, and the per-tile
        // chr_addr are constant for runs of 8 pixels in the same
        // tile. Hoist that work out of the per-pixel loop --
        // mirrors the BG tile-coherent restructure.
        uint16_t tile_y = oy >> 11;
        uint16_t chr_y  = (oy >> 8) & 7;
        int32_t  total  = rcx * 2;
        int32_t  px     = 0;

        while (px < total) {
            if (obj_x + px >= 240) break;

            uint16_t tile_x = ox >> 11;
            uint16_t chr_x  = (ox >> 8) & 7;

            uint32_t chr_addr = chr_base + tile_y * tys + chr_y * lsz;
            chr_addr += is_256 ? tile_x * 64 : tile_x * 32;

            uint8_t run = (pa > 0) ? (8 - chr_x) : (chr_x + 1);
            if (px + run > total) run = total - px;
            if (obj_x + px + run > 240) run = 240 - (obj_x + px);

            uint32_t pal_base = !is_256 ? chr_pal * 16 : 0;
            const uint8_t *src = vram + chr_addr;

            // Pre-decode the 8 palette indices for this tile row. See
            // the matching block in render_bg's tile-coherent path for
            // the rationale -- skips the per-pixel shift+mask in 4bpp.
            uint8_t row_pal[8];
            if (is_256) {
                row_pal[0] = src[0]; row_pal[1] = src[1];
                row_pal[2] = src[2]; row_pal[3] = src[3];
                row_pal[4] = src[4]; row_pal[5] = src[5];
                row_pal[6] = src[6]; row_pal[7] = src[7];
            } else {
                uint8_t b;
                b = src[0]; row_pal[0] = b & 0xf; row_pal[1] = b >> 4;
                b = src[1]; row_pal[2] = b & 0xf; row_pal[3] = b >> 4;
                b = src[2]; row_pal[4] = b & 0xf; row_pal[5] = b >> 4;
                b = src[3]; row_pal[6] = b & 0xf; row_pal[7] = b >> 4;
            }

            for (uint8_t i = 0; i < run; i++) {
                int16_t sx = obj_x + px + i;
                if (sx < 0) continue;

                uint8_t cx = (pa > 0) ? (chr_x + i) : (chr_x - i);

                uint8_t pal_idx = row_pal[cx];

                if (pal_idx
                    && (!windows_active || (layer_mask[sx] & 0x10))) {
                    uint32_t pal_addr = 0x100 | pal_idx | pal_base;
                    write_obj_pixel((uint8_t *)screen,
                                    address + i * 2,
                                    palette[pal_addr],
                                    (uint8_t)sx, obj_mode);
                }
            }

            px      += run;
            ox      += (int32_t)pa * run;
            address += run * 2;
        }
    } else {
        // Affine sprites: ox/oy advance by pa/pc/pb/pd per pixel
        // and don't have linear tile coherence, so keep the per-
        // pixel loop.
        for (int32_t x = 0; x < rcx * 2;
             x++, ox += pa, oy += pc, address += 2) {
            if (obj_x + x < 0)   continue;
            if (obj_x + x >= 240) break;

            uint16_t tile_x = ox >> 11;
            uint16_t tile_y = oy >> 11;

            if (ox < 0 || tile_x >= x_tiles) continue;
            if (oy < 0 || tile_y >= y_tiles) continue;

            uint16_t chr_x = (ox >> 8) & 7;
            uint16_t chr_y = (oy >> 8) & 7;

            uint32_t chr_addr = chr_base + tile_y * tys + chr_y * lsz;

            uint32_t vram_addr;
            uint32_t pal_idx;
            if (is_256) {
                vram_addr = chr_addr + tile_x * 64 + chr_x;
                pal_idx   = vram[vram_addr];
            } else {
                vram_addr = chr_addr + tile_x * 32 + (chr_x >> 1);
                pal_idx   = (vram[vram_addr] >> (chr_x & 1) * 4) & 0xf;
            }

            uint32_t pal_addr = 0x100 | pal_idx | (!is_256 ? chr_pal * 16 : 0);

            if (pal_idx
                && (!windows_active || (layer_mask[obj_x + x] & 0x10)))
                write_obj_pixel((uint8_t *)screen, address,
                                palette[pal_addr],
                                (uint8_t)(obj_x + x), obj_mode);
        }
    }
}

static void render_obj(uint8_t prio) {
    if (!(disp_cnt.w & OBJ_ENB)) return;

    uint8_t y = v_count.w;
    int n = obj_line_count[y][prio];

    // Iterate in REVERSE list order: highest OAM index pushed last,
    // drawn first (back); lowest pushed first, drawn last (front).
    // Matches the GBA's lower-OAM-on-top convention within a priority.
    for (int i = n - 1; i >= 0; i--) {
        uint8_t idx = obj_line_idx[y][prio][i];
        render_decoded_obj(&decoded_objs[idx]);
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

                            if (pal_idx
                                && (!windows_active
                                    || (layer_mask[x] & (1 << bg_idx))))
                                write_bg_pixel((uint8_t *)screen, address,
                                               palette[pal_idx], x,
                                               (uint8_t)bg_idx);
                        }
                    } else {
                        // Tile-coherent non-affine render. Consecutive pixels
                        // along a scanline almost always share a tile; we
                        // read the tilemap entry and unpack tile metadata
                        // (chr_numb, flip flags, palette select) ONCE per
                        // tile (8 pixels) instead of per pixel. Cuts the
                        // per-pixel work from ~30 SH4 ops to ~10.
                        uint16_t oy          = v_count.w + bg[bg_idx].yofs.w;
                        uint16_t tmy         = oy >> 3;
                        uint16_t scrn_y      = (tmy >> 5) & 1;
                        uint16_t base_chr_y  = oy & 7;
                        uint32_t row_in_screen = (tmy & 0x1f) * 32 * 2;

                        uint8_t x = 0;
                        uint16_t ox = bg[bg_idx].xofs.w;

                        while (x < 240) {
                            // Per-tile setup: tilemap address, tile entry,
                            // unpack flags, compute tile base in VRAM.
                            uint16_t tmx_full = ox >> 3;
                            uint16_t scrn_x   = (tmx_full >> 5) & 1;
                            uint16_t cx_start = ox & 7;

                            uint32_t map_addr = scrn_base + row_in_screen
                                              + (tmx_full & 0x1f) * 2;
                            switch (scrn_size) {
                                case 1: map_addr += scrn_x * 2048; break;
                                case 2: map_addr += scrn_y * 2048; break;
                                case 3: map_addr += scrn_x * 2048
                                                 + scrn_y * 4096; break;
                            }

                            uint16_t tile = vram[map_addr + 0]
                                          | (vram[map_addr + 1] << 8);
                            uint16_t chr_numb = (tile >>  0) & 0x3ff;
                            bool     flip_x   = (tile >> 10) & 0x1;
                            bool     flip_y   = (tile >> 11) & 0x1;
                            uint8_t  chr_pal  = (tile >> 12) & 0xf;

                            uint16_t chr_y = base_chr_y;
                            if (flip_y) chr_y ^= 7;

                            uint16_t pal_base = is_256 ? 0 : chr_pal * 16;

                            uint32_t tile_base = is_256
                                ? chr_base + chr_numb * 64 + chr_y * 8
                                : chr_base + chr_numb * 32 + chr_y * 4;

                            // Pre-decode all 8 palette indices for this
                            // tile row. The hot loop below is then a
                            // bare table lookup -- in 4bpp this lifts
                            // the per-pixel shift+mask out of the inner
                            // loop; in 8bpp it doesn't help much by
                            // itself but keeps the flip / window /
                            // pal_idx==0 branches uniform.
                            uint8_t row_pal[8];
                            const uint8_t *row = vram + tile_base;
                            if (is_256) {
                                row_pal[0] = row[0]; row_pal[1] = row[1];
                                row_pal[2] = row[2]; row_pal[3] = row[3];
                                row_pal[4] = row[4]; row_pal[5] = row[5];
                                row_pal[6] = row[6]; row_pal[7] = row[7];
                            } else {
                                uint8_t b;
                                b = row[0]; row_pal[0] = b & 0xf; row_pal[1] = b >> 4;
                                b = row[1]; row_pal[2] = b & 0xf; row_pal[3] = b >> 4;
                                b = row[2]; row_pal[4] = b & 0xf; row_pal[5] = b >> 4;
                                b = row[3]; row_pal[6] = b & 0xf; row_pal[7] = b >> 4;
                            }

                            // How many pixels of this tile fall on screen.
                            uint8_t pixels = 8 - cx_start;
                            if (x + pixels > 240) pixels = 240 - x;

                            uint8_t  bg_bit = 1 << bg_idx;
                            for (uint8_t i = 0; i < pixels; i++) {
                                uint16_t cx = cx_start + i;
                                if (flip_x) cx ^= 7;

                                uint8_t pal_idx = row_pal[cx];

                                if (pal_idx
                                    && (!windows_active
                                        || (layer_mask[x + i] & bg_bit))) {
                                    write_bg_pixel((uint8_t *)screen, address,
                                                   palette[pal_idx | pal_base],
                                                   (uint8_t)(x + i),
                                                   (uint8_t)bg_idx);
                                }
                                address += 2;
                            }

                            x  += pixels;
                            ox += pixels;
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

                uint16_t color = (r << 11) | (((g << 1) | (g >> 4)) << 5) | b;
                write_bg_pixel((uint8_t *)screen, surf_addr, color, x, LAYER_BG2);

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

                write_bg_pixel((uint8_t *)screen, surf_addr,
                               palette[pal_idx], x, LAYER_BG2);

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

    compute_blend_state();

    uint16_t backdrop = palette[0];
    // If backdrop is a brightness 1st-target, the displayed colour is
    // the brightened/darkened backdrop; alpha mode never applies to
    // backdrop alone (no second-target underneath). Pre-compute once
    // and broadcast across the line.
    if (blend_any && blend_mode >= 2 &&
        (blend_first & LAYER_BIT(LAYER_BD))) {
        backdrop = blend_apply(backdrop, LAYER_BD, backdrop, LAYER_BD, false);
    }
    if (blend_any) {
        for (int i = 0; i < 240; i++) {
            line[i] = backdrop;
            layer_id_buf[i] = LAYER_BD;
        }
    } else {
        for (int i = 0; i < 240; i++) line[i] = backdrop;
    }

    compute_layer_mask();

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
// frames.
//
// As of Pass 9.12, FRAMESKIP=0 (render every frame) gave no measurable
// slowdown vs. FRAMESKIP=1 in Minish Cap gameplay -- arm_exec is the
// bottleneck, render+dupdate fits comfortably in the per-frame budget.
// Kept as a build switch in case a render-heavier game emerges.
//   FRAMESKIP=0: render every frame   (current default)
//   FRAMESKIP=1: render every other
//   FRAMESKIP=2: render 1 in 3
#ifndef FRAMESKIP
#define FRAMESKIP 0
#endif

void run_frame() {
    static uint32_t skip_phase = 0;
    bool render_this_frame = (skip_phase++ % (FRAMESKIP + 1)) == 0;

    // Decide once per frame whether the per-scanline CPU schedule needs
    // the mid-line exit. The split exists so HBlank IRQ and HBlank-timed
    // DMA fire between the visible-portion arm_exec and the HBlank-
    // portion arm_exec. If neither is active, the second slice is pure
    // overhead -- cut it out and run a single arm_exec(CYC_LINE_TOTAL)
    // per line, halving the per-frame CPU-slice count from ~456 to ~228.
    //
    // Detection is pinned at frame start. A cart that enables HBlank
    // IRQ mid-frame would have IRQs skipped until next frame; carts
    // almost always configure HBlank IRQ during VBlank or boot, so this
    // is a tolerable edge case.
    bool hblank_irq_on = (disp_stat.w & HBLK_IRQ) != 0;
    bool hblank_dma_on = false;
    for (int ch = 0; ch < 4; ch++) {
        if ((dma_ch[ch].ctrl.w & DMA_ENB) &&
            ((dma_ch[ch].ctrl.w >> 12) & 3) == HBLANK) {
            hblank_dma_on = true;
            break;
        }
    }
    bool need_hblank_split = hblank_irq_on || hblank_dma_on;

    // Decode OAM into per-(scanline, priority) sprite lists once for
    // this frame. Skipped when frame is being skipped -- nothing reads
    // the lists in that case.
    if (render_this_frame) build_sprite_lists();

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

        if (need_hblank_split) {
            // Split path: HBlank effects are active this frame. Run CPU
            // up to the HBlank boundary, render, fire HBlank DMA + IRQ,
            // then run CPU through HBlank.
            BENCH_TIME_BLOCK(bench_arm_exec_ticks, arm_exec(CYC_LINE_HBLK0));

            if (v_count.w < LINES_VISIBLE) {
                if (render_this_frame) {
                    BENCH_TIME_BLOCK(bench_render_ticks, render_line());
                }
                dma_transfer_gba(HBLANK);
            }

            hblank_start();

            BENCH_TIME_BLOCK(bench_arm_exec_ticks, arm_exec(CYC_LINE_HBLK1));
        } else {
            // Merged path: no HBlank IRQ, no HBlank-timed DMA. Render
            // with start-of-line register state (closer to real-hardware
            // tile-mode "snapshot per scanline" than the original's
            // mid-line snapshot, and the cart can't react in real time
            // anyway with no IRQ to wake on). Then run a single full-
            // line arm_exec.
            //
            // hblank_start() still fires so polling carts see HBLK_FLAG
            // transition; no IRQ is triggered because HBLK_IRQ is off
            // in this branch.
            if (v_count.w < LINES_VISIBLE && render_this_frame) {
                BENCH_TIME_BLOCK(bench_render_ticks, render_line());
            }
            // dma_transfer_gba(HBLANK) skipped: no HBlank-timed
            // channels are enabled in this branch.

            hblank_start();

            BENCH_TIME_BLOCK(bench_arm_exec_ticks, arm_exec(CYC_LINE_TOTAL));
        }

        // Sound is not piped to any output yet, and sound_clock() does
        // double-precision FP math per scanline that is very slow on the
        // SH4 (no hardware FPU usage by gint). Skip it for now.
        // sound_clock(CYC_LINE_TOTAL);
    }

    // If we're skipping the render+dupdate this frame, return now. The
    // diagnostic overlay below would otherwise update gint_vram and trigger
    // the dupdate that we're trying to avoid.
    if (!render_this_frame) return;

#if GBA_DEBUG_OVERLAY
    // Frame heartbeat + diagnostic overlay. Lives in the white margin below
    // the centered GBA viewport so it never collides with the emulated frame.
    {
        extern io_reg int_enb, int_ack, int_enb_m;
        (void)int_enb_m;

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

        // Pack lazy NZCV into cpsr so the dump shows current flags.
        arm_flags_to_cpsr();

        dprint(2, 196, C_BLACK,
               "F:%lu PC:%08lX op:%02X%02X%02X%02X DISPCNT:%04X PAL0:%04X",
               (unsigned long)frame_count,
               (unsigned long)pc,
               b0, b1, b2, b3,
               (unsigned)disp_cnt.w,
               (unsigned)palette[0]);

        // Show the bench's measured frame time + derived fps, so we can
        // visually verify whether bench numbers match perceived speed.
        // last_frame_us is the previous frame's wall-clock measurement
        // (this frame's overlay is rendered before its own frame_us is
        // computed, so we always lag by 1 frame -- fine for diagnostics).
        uint32_t flu = bench_last_frame_us;
        uint32_t fps10 = (flu > 0) ? (10000000u / flu) : 0;  // fps x 10
        dprint(2, 210, C_BLACK,
               "%lu us/f  %lu.%lu fps  halt:%d IE:%04X IF:%04X",
               (unsigned long)flu,
               (unsigned long)(fps10 / 10),
               (unsigned long)(fps10 % 10),
               (int)int_halt,
               (unsigned)int_enb.w,
               (unsigned)int_ack.w);

        frame_count++;
    }
#endif // GBA_DEBUG_OVERLAY

    // gint_vram is the active framebuffer; just push it to the display.
    BENCH_TIME_BLOCK(bench_dupdate_ticks, dupdate());

    // sound_buffer_wrap();   // see note above on sound_clock()
}