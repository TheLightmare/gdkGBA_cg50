// GINT LIBS =====
#include <gint/display.h>
#include <gint/keyboard.h>
#include <gint/kmalloc.h>
#include <gint/bfile.h>
#include <gint/gint.h>

#include "build_flags.h"
#if GBA_FXLINK
#  include <gint/usb.h>
#  include <gint/usb-ff-bulk.h>
#endif

#include "gint_gba.h"
//===============

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "arm.h"
#include "arm_mem.h"
#include "bench.h"
#include "build_flags.h"
#include "mem_swizzle.h"
#include "rom_buffer.h"
#include "save_file.h"
#include "arm_block.h"
#include "thumb_block.h"
#include "extram.h"

#include "io.h"

#include "video.h"


const int64_t max_rom_sz = 32 * 1024 * 1024;

// Scan storage memory for the first .gba file. On success, writes the
// basename (e.g. "Pokemon.gba") into `out` and returns true. Falls back
// to "test.gba" so existing setups keep working.
static bool find_first_gba_rom(char *out, size_t outsize) {
    int handle;
    uint16_t found[256];
    struct BFile_FileInfo info;

    int r = BFile_FindFirst(u"\\\\fls0\\*.gba", &handle, found, &info);
    if (r < 0) {
        // No matching file found -- fall back to legacy default.
        snprintf(out, outsize, "test.gba");
        return false;
    }
    BFile_FindClose(handle);

    // FONTCHARACTER -> char (ASCII subset). Filenames in user storage are
    // basically ASCII; characters above 0x7F would be Casio-specific
    // multi-byte sequences but those don't appear in normal .gba names.
    size_t len = 0;
    while (found[len] != 0 && len < outsize - 1) {
        out[len] = (char)(found[len] & 0xFF);
        len++;
    }
    out[len] = 0;
    return true;
}

static uint32_t to_pow2(uint32_t val) {
    val--;

    val |= (val >>  1);
    val |= (val >>  2);
    val |= (val >>  4);
    val |= (val >>  8);
    val |= (val >> 16);

    return val + 1;
}

// 20 KB backup buffer for gint's on-chip RAM save/restore on world switch.
// We place hot interpreter state (arm_r, lazy flags, etc.) in XYRAM for
// fast deterministic access; world switches (e.g. fopen during ROM chunk
// load) would otherwise corrupt that state. With BACKUP mode gint copies
// XYRAM/ILRAM into this buffer on switch-out and restores on switch-in.
static uint8_t onchip_backup_buf[GINT_ONCHIP_BUFSIZE];

// Bounds-safe snprintf accumulator for the diagnostic snap buffer. The
// naive `n += snprintf(buf+n, sizeof(buf)-n, ...)` pattern overshoots
// sizeof(buf) when a format would have produced more than the remaining
// space -- snprintf returns "would have written" not "did write" -- and
// the next call then does an unsigned underflow on the size argument and
// writes past the buffer end. Clamp n to bufsize-1 after every call.
static size_t safe_append(char *buf, size_t bufsize, size_t n,
                          const char *fmt, ...) {
    if (n >= bufsize) return n;
    va_list ap;
    va_start(ap, fmt);
    int r = vsnprintf(buf + n, bufsize - n, fmt, ap);
    va_end(ap);
    if (r > 0) {
        n += (size_t)r;
        if (n >= bufsize) n = bufsize - 1;
    }
    return n;
}

int main(void) {
    // Set up on-chip RAM backup BEFORE any code that might world-switch
    // (file I/O during BIOS/ROM init, etc.). Must happen before any code
    // uses XYRAM-placed globals.
    gint_set_onchip_save_mode(GINT_ONCHIP_BACKUP, onchip_backup_buf);

    // Reserve a TMU for free-running phase timing.
    bench_init();

    // Probe and register the optional extram arena BEFORE any heap-heavy
    // initialisation. ewram_init() will then prefer extram over the BSS
    // fallback for the 256 KB EWRAM allocation.
    extram_init();
    ewram_init();

#if GBA_DIAG_LOG
    dclear(C_WHITE);
    dtext (1, 20, C_BLACK, "gdkGBA - Gameboy Advance emulator made by gdkchan");
    dtext (1, 40, C_BLACK, "ported to fx-CG50 by Lightmare");
    dtext (1, 60, C_BLACK, "This is FREE software released into the PUBLIC DOMAIN");
    dprint(1, 90, C_BLACK, "%s", extram_status);
    dprint(1,110, C_BLACK, "EWRAM: %u KB (mask 0x%05X)",
           (unsigned)(ewram_size / 1024), (unsigned)ewram_mask);
    dupdate();

    getkey();
#endif

    gint_gba_init();
    arm_init();

    // Allocate the Thumb block cache. Failure is not fatal -- the
    // interpreter runs unchanged when thumb_block_enabled stays false.
    thumb_block_init();
    arm_block_init();

    // Allocate the fit-to-screen source backup buffer (76 KB, prefers
    // extram). Failure is silent; fit mode just stays a no-op.
    gba_fit_init();


    //TODO : make an actual ROM select menu
    
    /* if (argc < 2) {
        printf("Error: Invalid number of arguments!\n");
        printf("Please specify a ROM file.\n");

        return 0;
    } */


    FILE *image;

    image = fopen("gba_bios.bin", "rb");

    if (image == NULL) {
        //printf("Error: GBA BIOS not found!\n");
        //printf("Place it on this directory with the name \"gba_bios.bin\".\n");
        dclear(C_WHITE);
        dtext(1, 1, C_BLACK, "Error: GBA BIOS not found!");
        dtext(1, 20, C_BLACK, "Place it on this directory with the name \"gba_bios.bin\".");
        dupdate();

        getkey();

        return 0;
    }

    size_t bios_read = fread(bios, 1, 16384, image);
    (void)bios_read;

    fclose(image);

#if GBA_DIAG_LOG
    // Print BIOS bytes at the addresses where the ARM core has been seen
    // looping (0x00, 0x18 = IRQ vector, 0x28, 0x2C). Word-decoded for ARM.
    dclear(C_WHITE);
    dprint(1,   1, C_BLACK, "BIOS bytes read: %u", (unsigned)bios_read);
    dprint(1,  20, C_BLACK, "0x00:%02X%02X%02X%02X 0x04:%02X%02X%02X%02X",
           bios[0], bios[1], bios[2], bios[3],
           bios[4], bios[5], bios[6], bios[7]);
    dprint(1,  40, C_BLACK, "0x18:%02X%02X%02X%02X 0x1C:%02X%02X%02X%02X",
           bios[0x18], bios[0x19], bios[0x1A], bios[0x1B],
           bios[0x1C], bios[0x1D], bios[0x1E], bios[0x1F]);
    dprint(1,  60, C_BLACK, "0x20:%02X%02X%02X%02X 0x24:%02X%02X%02X%02X",
           bios[0x20], bios[0x21], bios[0x22], bios[0x23],
           bios[0x24], bios[0x25], bios[0x26], bios[0x27]);
    dprint(1,  80, C_BLACK, "0x28:%02X%02X%02X%02X 0x2C:%02X%02X%02X%02X",
           bios[0x28], bios[0x29], bios[0x2A], bios[0x2B],
           bios[0x2C], bios[0x2D], bios[0x2E], bios[0x2F]);
    dupdate();
    getkey();
#endif

    // Find the first .gba file in storage memory. Fall back to "test.gba"
    // so existing setups still work without renaming.
    char rom_path[64];
    bool rom_was_found = find_first_gba_rom(rom_path, sizeof(rom_path));

    image = fopen(rom_path, "rb");

    if (image == NULL) {
        dclear(C_WHITE);
        dtext(1, 1, C_BLACK, "Error: ROM file couldn't be opened.");
        dprint(1, 20, C_BLACK, "Tried: %s", rom_path);
        dtext(1, 40, C_BLACK, rom_was_found
            ? "(file was found by directory scan)"
            : "(no .gba files in storage; place one alongside fxgba.g3a)");
        dupdate();

        getkey();

        return 0;
    }

#if GBA_DIAG_LOG
    dclear(C_WHITE);
    dtext(1,  1, C_BLACK, "loading ROM image:");
    dprint(1, 20, C_BLACK, "%s", rom_path);
    dtext(1, 40, C_BLACK, rom_was_found
        ? "(found by directory scan)"
        : "(fallback to test.gba)");
    dupdate();
    getkey();
#endif

    fseek(image, 0, SEEK_END);

    cart_rom_size = ftell(image);

#if GBA_DIAG_LOG
    dclear(C_WHITE);
    dprint(1, 1, C_BLACK, "cart_rom_size : %ld bytes", cart_rom_size);
    dupdate();
    getkey();
#endif

    cart_rom_mask = to_pow2(cart_rom_size) - 1;

    if (cart_rom_size > max_rom_sz) cart_rom_size = max_rom_sz;

    // Close the file handle - we'll reopen it in the buffer system
    fclose(image);
    
    // Initialize the ROM buffer system instead of loading the entire ROM
    rom_init_status_e rb_status;
    int rb_failed_chunk;
    if (!rom_buffer_init(&rom_buffer, rom_path, cart_rom_mask,
                         &rb_status, &rb_failed_chunk)) {
        // Probe the largest currently-available block in the user-RAM arena so
        // we can tell whether the failure is heap exhaustion or something else.
        size_t largest = 0;
        void *probe = kmalloc_max(&largest, "_uram");
        if (probe) kfree(probe);

        dclear(C_WHITE);
        dtext(1, 1, C_BLACK, "Error initializing ROM buffer system");

        switch (rb_status) {
            case ROM_INIT_FOPEN_FAILED:
                dprint(1, 20, C_BLACK, "fopen(\"%s\") failed", rom_path);
                break;
            case ROM_INIT_MALLOC_FAILED:
                dprint(1, 20, C_BLACK,
                       "malloc failed at chunk %d/%d (each %d KB)",
                       rb_failed_chunk, ROM_BUFFER_CHUNKS,
                       ROM_CHUNK_SIZE / 1024);
                break;
            default:
                dtext(1, 20, C_BLACK, "Unknown error");
                break;
        }

        dprint(1, 40, C_BLACK, "Largest free block (_uram): %u bytes",
               (unsigned)largest);

        dupdate();
        getkey();
        return 0;
    }

#if GBA_DIAG_LOG
    dclear(C_WHITE);
    dtext (1,  1, C_BLACK, "Successfully initialized ROM buffer!");
    dprint(1, 20, C_BLACK, "active chunks: %d/%d  (each %d KB)",
           rom_buffer.active_chunks, ROM_BUFFER_CHUNKS, ROM_CHUNK_SIZE / 1024);
    dprint(1, 40, C_BLACK, "%s", extram_status);
    dupdate();
    getkey();
#endif

    arm_reset();

    // Derive the .sav path from the ROM path and try to load any existing
    // save file. Failure is silent: a missing or malformed file just means
    // the game starts with a blank save, which is the desired behaviour for
    // first-time runs.
    char save_path[80];
    bool have_save_path =
        save_make_path(rom_path, save_path, sizeof(save_path));
    if (have_save_path) save_load(save_path);

#if GBA_FXLINK
    // Bring up the USB bulk interface for fxlink screen streaming. This
    // MUST happen after all init-time fopen/fread (BIOS, ROM, save) --
    // those world-switch to Casio OS, which clobbers gint's USB pipe state
    // and causes the device to renumerate after the first transfer.
    // Subsequent world switches (rom-buffer chunk loads, periodic save
    // flushes) will still cause occasional disconnects; running fxlink
    // with -r lets it auto-reconnect.
    usb_interface_t const *interfaces[] = { &usb_ff_bulk, NULL };
    usb_open(interfaces, GINT_CALL_NULL);
    usb_open_wait();
#endif


#if GBA_DIAG_LOG
    // One-shot BIOS + cart dump to log file at boot.
    {
        FILE *log = fopen("fxgba_log.txt", "w");
        if (log) {
            fprintf(log, "=== fxgba diagnostic log ===\n");
            fprintf(log, "build: %s %s tag=43 (LE-fetch fix)\n",
                    __DATE__, __TIME__);
            // Runtime endian probe: write a known value and inspect bytes.
            { union { uint32_t w; uint8_t b[4]; } u;
              u.w = 0x01020304;
              fprintf(log, "endian probe: w=01020304 -> b[]=%02X %02X %02X %02X (host is %s)\n",
                      u.b[0], u.b[1], u.b[2], u.b[3],
                      (u.b[0] == 0x01) ? "BE" : (u.b[0] == 0x04) ? "LE" : "MIXED");
#ifdef __BYTE_ORDER__
              fprintf(log, "__BYTE_ORDER__=%d  __ORDER_LITTLE_ENDIAN__=%d  __ORDER_BIG_ENDIAN__=%d\n",
                      __BYTE_ORDER__, __ORDER_LITTLE_ENDIAN__, __ORDER_BIG_ENDIAN__);
#endif
            }
            // Show the live POSTFLG so we can confirm this build's setting.
            extern uint8_t post_boot;
            fprintf(log, "POSTFLG at boot: %u\n", (unsigned)post_boot);
            fprintf(log, "BIOS bytes read: %u\n", (unsigned)bios_read);

            // Dump key Thumb decoder slots. If t16_blx_h1 was correctly
            // unregistered, slots 0x740-0x77F should equal arm_und.
            extern void (*thumb_proc[2048])();
            // arm_und is static, but we can look at its slot indirectly:
            // the default-fill leaves *all* unmapped slots equal, so any
            // slot we know we never registered (e.g. 0b11101100000 = 0x760
            // which is NEVER mapped, even before the fix) is arm_und.
            void *arm_und_addr = (void *)thumb_proc[0x760];
            fprintf(log, "arm_und addr = %p\n", arm_und_addr);
            fprintf(log, "thumb[0x740] = %p (E800 BLX suffix; should == arm_und)\n",
                    (void *)thumb_proc[0x740]);
            fprintf(log, "thumb[0x750] = %p (EA00; should == arm_und)\n",
                    (void *)thumb_proc[0x750]);
            fprintf(log, "thumb[0x77F] = %p (EFFE; should == arm_und)\n",
                    (void *)thumb_proc[0x77F]);
            fprintf(log, "thumb[0x780] = %p (F000 BL prefix; != arm_und)\n",
                    (void *)thumb_proc[0x780]);
            fprintf(log, "thumb[0x7C0] = %p (F800 BL suffix; != arm_und)\n",
                    (void *)thumb_proc[0x7C0]);
            fprintf(log, "thumb[0x238] = %p (4700 BX; != arm_und)\n",
                    (void *)thumb_proc[0x238]);
            fprintf(log, "thumb[0x23C] = %p (4780 BLX reg; should == arm_und)\n",
                    (void *)thumb_proc[0x23C]);
            fprintf(log, "\n");
            // First 0x140 bytes (vectors + IRQ handler + FIQ shared code).
            for (int i = 0; i < 0x140; i += 16) {
                fprintf(log, "%04X:", i);
                for (int j = 0; j < 16; j++) fprintf(log, " %02X", bios[i + j]);
                fprintf(log, "\n");
            }
            // SWI dispatcher area.
            fprintf(log, "\nSWI dispatcher (0x140):\n");
            for (int i = 0x140; i < 0x180; i += 16) {
                fprintf(log, "%04X:", i);
                for (int j = 0; j < 16; j++) fprintf(log, " %02X", bios[i + j]);
                fprintf(log, "\n");
            }
            fprintf(log, "\n");
            fclose(log);
        }
    }

    {
        extern volatile uint8_t arm_first_low_set;
        arm_first_low_set = 0;  // Ignore the arm_reset path; only record real
                                // mid-execution branches into BIOS region.
    }

    // After arm_reset the rom_buffer chunks are already populated (from the
    // first cart fetch). Dump a small bounded range around the address where
    // the cart is known to spin, so we can decode the loop. Small enough not
    // to thrash the chunk cache.
    {
        FILE *log = fopen("fxgba_log.txt", "a");
        if (log) {
            fprintf(log, "Cart 0x08040BE0..0x08040C20:\n");
            for (int i = 0; i < 0x40; i++) {
                if (i % 16 == 0) fprintf(log, "%08X:", 0x08040BE0 + i);
                fprintf(log, " %02X", arm_readb(0x08040BE0 + i));
                if (i % 16 == 15) fprintf(log, "\n");
            }
            fprintf(log, "\n");
            fclose(log);
        }
    }
#endif // GBA_DIAG_LOG

    // Wipe gint_vram once before starting the emulation loop so leftover
    // boot-screen text outside the centered GBA viewport doesn't bleed
    // into the displayed frame. The viewport itself gets overwritten every
    // frame by render_line() so this only affects the margins.
    dclear(C_BLACK);
    dupdate();

    bool run = true;

    // Auto-save bookkeeping. save_dirty is set by writes into the save
    // region (see save_mark_dirty in arm_mem.c). We debounce: each time we
    // observe dirty we re-arm a quiescence countdown, and only flush after
    // SAVE_QUIESCENT_FRAMES of no further activity. This collapses the
    // burst of writes during an in-game save into one disk write, and
    // avoids hitting storage on every byte the game flushes.
    uint32_t save_frame             = 0;
    uint32_t save_last_dirty_frame  = 0;
    bool     save_pending           = false;
    const uint32_t SAVE_QUIESCENT_FRAMES = 60;

#if GBA_DIAG_LOG
    uint32_t loop_frame = 0;
    int dumps_done = 0;
    int spike_dumps = 0;
    int manual_dumps = 0;
    uint32_t last_spike_frame = 0;
    bool f3_was_down = false;
    bool capture_now_pending = false;
    // Steady-state runs ~90 ms/frame on Ptune4 F5; flag anything over 200 ms
    // as a spike worth investigating. Cap spike snapshots aggressively and
    // require a frame gap between captures: a fade animation generates one
    // slow frame every 2 frames, so without gating we'd hammer the
    // filesystem for many seconds and risk a crash (saw this once).
    const uint32_t SPIKE_THRESHOLD_US = 200000;
    const int MAX_SPIKE_DUMPS = 5;
    const uint32_t SPIKE_MIN_GAP = 30;
    const int MAX_MANUAL_DUMPS = 20;
#endif

    while (run) {
#if GBA_BENCH
        // Capture per-frame timing BEFORE run_frame so we can compute the
        // single-frame deltas of each phase, regardless of when (and whether)
        // the cumulative bench_*_ticks accumulators get reset by a snapshot.
        uint32_t frame_t0 = bench_now();
        uint64_t arm_at_start = bench_arm_exec_ticks;
        uint64_t ren_at_start = bench_render_ticks;
        uint64_t dup_at_start = bench_dupdate_ticks;
        uint32_t slr_at_start = bench_mem_slow_read;
        uint32_t slw_at_start = bench_mem_slow_write;
        uint32_t cm_at_start  = bench_chunk_miss;
        uint64_t armm_at_start = bench_arm_mode_ticks;
        uint64_t thbm_at_start = bench_thumb_mode_ticks;
        uint32_t acs_at_start = bench_arm_cond_skip;
        uint32_t ass_at_start = bench_arm_ss_inst;
        uint32_t abi_at_start = bench_arm_block_inst;
        uint32_t abd_at_start = bench_arm_block_decodes;
        uint32_t tss_at_start = bench_thumb_ss_inst;
        uint32_t tbi_at_start = bench_thumb_block_inst;
        uint32_t tbd_at_start = bench_thumb_block_decodes;
#endif

        run_frame();

#if GBA_BENCH
        // Per-frame elapsed in us.
        uint32_t frame_us = bench_freq_hz
            ? (uint32_t)((uint64_t)bench_elapsed(frame_t0) * 1000000ULL / bench_freq_hz)
            : 0;
        // Publish for the on-screen heartbeat overlay (lets the user
        // visually compare what bench thinks vs. what they perceive).
        bench_last_frame_us = frame_us;
#endif

        // Debounced auto-save. The write paths set save_dirty; we re-arm
        // the quiescence window on each observation. When no save-region
        // writes have happened for SAVE_QUIESCENT_FRAMES we flush. The
        // disk write happens between guest frames, so the worst-case stall
        // lands in idle time rather than mid-scene.
        save_frame++;
        if (save_dirty) {
            save_dirty = false;
            save_last_dirty_frame = save_frame;
            save_pending = true;
        }
        if (save_pending && have_save_path &&
            (save_frame - save_last_dirty_frame) >= SAVE_QUIESCENT_FRAMES) {
            save_write(save_path);
            save_pending = false;
        }

        clearevents();

        if (keydown(KEY_MENU) || keydown(KEY_EXIT)) {
            run = false;
            continue;
        }

        // F2 toggles fit-to-screen mode. Edge-detect so a held key
        // only flips once. On transition we wipe gint_vram so leftover
        // pixels from the previous mode's larger region don't bleed
        // into the new mode's margins.
        static bool f2_was_down = false;
        bool f2_now = keydown(KEY_F2);
        if (f2_now && !f2_was_down) {
            gba_fit_to_screen = !gba_fit_to_screen;
            dclear(C_BLACK);
            dupdate();
        }
        f2_was_down = f2_now;

#if GBA_DIAG_LOG
        // F3 = manual bench snapshot. Edge-detect so a held key only
        // captures once. Use this to grab a log mid-gameplay where the
        // scheduled snapshots can't reach (e.g., past frame 3600, or in
        // a specific scene that's slow).
        bool f3_now = keydown(KEY_F3);
        if (f3_now && !f3_was_down && manual_dumps < MAX_MANUAL_DUMPS) {
            capture_now_pending = true;
        }
        f3_was_down = f3_now;
#endif

        uint16_t pressed = 0;
        if (keydown(KEY_UP))    pressed |= BTN_U;
        if (keydown(KEY_DOWN))  pressed |= BTN_D;
        if (keydown(KEY_LEFT))  pressed |= BTN_L;
        if (keydown(KEY_RIGHT)) pressed |= BTN_R;
        if (keydown(KEY_SHIFT)) pressed |= BTN_A;
        if (keydown(KEY_ALPHA)) pressed |= BTN_B;
        if (keydown(KEY_F1))    pressed |= BTN_LT;
        if (keydown(KEY_F6))    pressed |= BTN_RT;
        if (keydown(KEY_OPTN))  pressed |= BTN_SEL;
        if (keydown(KEY_EXE))   pressed |= BTN_STA;

        key_input.w = 0x3ff & ~pressed;

#if GBA_DIAG_LOG
        // Capture diagnostic snapshots into a single in-memory buffer first,
        // then dump in one fwrite. fprintf-by-piece in gint world has been
        // crashing the calculator. Snapshots fire on a fixed schedule so we
        // get coverage from boot through long-running steady state. Each one
        // also reports per-frame deltas plus cumulative-since-last-snap.
        //
        // Additionally, any frame whose total time exceeds SPIKE_THRESHOLD_US
        // triggers an extra "spike" snapshot, capped at MAX_SPIKE_DUMPS to
        // avoid log spam if the slowdown is sustained.
        loop_frame++;
        static const uint32_t snapshot_frames[] = {
            1, 5, 30, 60, 120, 240, 480, 900, 1500, 2400, 3600,
            5400, 7200, 10800
        };
        const int SNAPSHOT_FRAMES_COUNT =
            (int)(sizeof(snapshot_frames) / sizeof(snapshot_frames[0]));
        bool is_scheduled = (dumps_done < SNAPSHOT_FRAMES_COUNT)
                         && (loop_frame == snapshot_frames[dumps_done]);
        // Skip spike detection during boot frames (frame 1 is naturally huge
        // because of BIOS/cart init). Only flag spikes after we're settled,
        // require a minimum gap since the last spike, and cap the total
        // count — fade animations and other sustained slow phases generate
        // many spikes in a row and we don't want to spam disk writes.
        bool is_spike = !is_scheduled
                     && (loop_frame > 60)
                     && (frame_us > SPIKE_THRESHOLD_US)
                     && (spike_dumps < MAX_SPIKE_DUMPS)
                     && (loop_frame - last_spike_frame >= SPIKE_MIN_GAP);
        bool is_manual = capture_now_pending && !is_scheduled && !is_spike;
        capture_now_pending = false;
        if (is_scheduled || is_spike || is_manual) {
            char buf[4096];
            size_t n = 0;
            arm_flags_to_cpsr();
            if (is_spike) {
                n = safe_append(buf, sizeof(buf), n,
                    "--- SPIKE %d (frame %lu, this frame=%lu us) ---\n",
                    spike_dumps, (unsigned long)loop_frame,
                    (unsigned long)frame_us);
            } else if (is_manual) {
                n = safe_append(buf, sizeof(buf), n,
                    "--- MANUAL %d (frame %lu, this frame=%lu us) ---\n",
                    manual_dumps, (unsigned long)loop_frame,
                    (unsigned long)frame_us);
            } else {
                n = safe_append(buf, sizeof(buf), n,
                    "--- snap %d (frame %lu, this frame=%lu us) ---\n",
                    dumps_done, (unsigned long)loop_frame,
                    (unsigned long)frame_us);
            }
            n = safe_append(buf, sizeof(buf), n,
                "PC=%08lX CPSR=%08lX halt=%d\n",
                (unsigned long)arm_r.r[15],
                (unsigned long)arm_r.cpsr,
                (int)int_halt);
            n = safe_append(buf, sizeof(buf), n,
                "IE=%04X IF=%04X IME=%X DISPCNT=%04X PAL0=%04X\n",
                (unsigned)int_enb.w, (unsigned)int_ack.w,
                (unsigned)(int_enb_m.w & 1),
                (unsigned)disp_cnt.w, (unsigned)palette[0]);
            for (int i = 0; i < 16; i += 4) {
                n = safe_append(buf, sizeof(buf), n,
                    "r%d-r%d: %08lX %08lX %08lX %08lX\n",
                    i, i + 3,
                    (unsigned long)arm_r.r[i + 0],
                    (unsigned long)arm_r.r[i + 1],
                    (unsigned long)arm_r.r[i + 2],
                    (unsigned long)arm_r.r[i + 3]);
            }
            // 32 bytes around current PC: 8 before, 24 from PC onwards.
            // For BIOS reads from bios[] directly. For cart reads via
            // rom_buffer through arm_readb -- this triggers at most one
            // chunk load since the snapshot only fires every 30 frames.
            uint32_t pc = arm_r.r[15] & ~1u;
            uint32_t base = pc >= 8 ? pc - 8 : pc;
            n = safe_append(buf, sizeof(buf), n, "@%08lX:",
                (unsigned long)base);
            for (int j = 0; j < 32; j++) {
                uint8_t b;
                uint32_t a = base + j;
                if (a < 0x4000)             b = bios[a];
                else if ((a >> 24) == 2)    b = mem_swz_read_b(wram_board, a & ewram_mask);
                else if ((a >> 24) == 3)    b = mem_swz_read_b(wram_chip,  a & 0x7FFF);
                else if ((a >> 24) >= 8 && (a >> 24) <= 0xB)
                                            b = arm_readb(a);
                else                        b = 0;
                if (j == 8) n = safe_append(buf, sizeof(buf), n, " >");
                n = safe_append(buf, sizeof(buf), n, " %02X", b);
            }
            n = safe_append(buf, sizeof(buf), n, "\n");

            // Sparse PC trace from this frame (every 4096th instruction).
            extern uint32_t arm_pc_trace[16], arm_pc_trace_pos;
            extern volatile uint32_t arm_undef_count;
            extern volatile uint32_t arm_undef_first_pc;
            extern volatile uint32_t arm_undef_first_op;
            extern volatile uint32_t arm_first_low_pc;
            extern volatile uint32_t arm_first_low_lr;
            extern volatile uint32_t arm_first_low_op;
            extern volatile uint8_t  arm_first_low_was_thumb;
            extern volatile uint32_t arm_swi_count[256];
            extern volatile uint8_t  arm_swi_last;
            extern volatile uint32_t arm_hle_irq_count;

            // Read what BIOS will fetch as the user IRQ handler address.
            // The BIOS at 0x134 does LDR pc,[0x03FFFFFC], and that mirrors
            // wram_chip[0x7FFC..0x7FFF] (IWRAM, 32 KB masked).
            uint32_t user_irq_handler = mem_swz_read_w(wram_chip, 0x7FFC);
            n = safe_append(buf, sizeof(buf), n,
                "*(0x03007FFC) = %08lX  (cart user IRQ handler)\n",
                (unsigned long)user_irq_handler);

            n = safe_append(buf, sizeof(buf), n,
                "undef=%lu  1st-low PC=%08lX LR=%08lX op=%08lX thumb=%d\n"
                "SWI last=%02X 01=%lu 02=%lu 04=%lu 05=%lu 06=%lu "
                "0B=%lu 0C=%lu 0E=%lu 0F=%lu 11=%lu 12=%lu 13=%lu 14=%lu 15=%lu\n",
                (unsigned long)arm_undef_count,
                (unsigned long)arm_first_low_pc,
                (unsigned long)arm_first_low_lr,
                (unsigned long)arm_first_low_op,
                (int)arm_first_low_was_thumb,
                (unsigned)arm_swi_last,
                (unsigned long)arm_swi_count[0x01],
                (unsigned long)arm_swi_count[0x02],
                (unsigned long)arm_swi_count[0x04],
                (unsigned long)arm_swi_count[0x05],
                (unsigned long)arm_swi_count[0x06],
                (unsigned long)arm_swi_count[0x0B],
                (unsigned long)arm_swi_count[0x0C],
                (unsigned long)arm_swi_count[0x0E],
                (unsigned long)arm_swi_count[0x0F],
                (unsigned long)arm_swi_count[0x11],
                (unsigned long)arm_swi_count[0x12],
                (unsigned long)arm_swi_count[0x13],
                (unsigned long)arm_swi_count[0x14],
                (unsigned long)arm_swi_count[0x15]);
            n = safe_append(buf, sizeof(buf), n,
                "HLE IRQ entries (cumulative): %lu\n",
                (unsigned long)arm_hle_irq_count);
            n = safe_append(buf, sizeof(buf), n, "PC trace (newest last):\n");
            uint32_t start = arm_pc_trace_pos > 16
                ? arm_pc_trace_pos - 16 : 0;
            for (uint32_t k = start; k < arm_pc_trace_pos; k++) {
                n = safe_append(buf, sizeof(buf), n, " %08lX",
                    (unsigned long)arm_pc_trace[k & 15]);
                if ((k - start) % 4 == 3)
                    n = safe_append(buf, sizeof(buf), n, "\n");
            }
            n = safe_append(buf, sizeof(buf), n, "\n");

            // Per-frame breakdown for THIS frame. Useful for spike snapshots
            // (the whole point is to see what's slow on the spike frame),
            // and provides a fresh datapoint on scheduled snapshots too.
            uint32_t fa_us = bench_freq_hz
                ? (uint32_t)(((bench_arm_exec_ticks - arm_at_start) * 1000000ULL) / bench_freq_hz)
                : 0;
            uint32_t fr_us = bench_freq_hz
                ? (uint32_t)(((bench_render_ticks - ren_at_start) * 1000000ULL) / bench_freq_hz)
                : 0;
            uint32_t fd_us = bench_freq_hz
                ? (uint32_t)(((bench_dupdate_ticks - dup_at_start) * 1000000ULL) / bench_freq_hz)
                : 0;
            uint32_t fslr = bench_mem_slow_read  - slr_at_start;
            uint32_t fslw = bench_mem_slow_write - slw_at_start;
            uint32_t fcm  = bench_chunk_miss     - cm_at_start;
            uint32_t farm_us = bench_freq_hz
                ? (uint32_t)(((bench_arm_mode_ticks - armm_at_start) * 1000000ULL) / bench_freq_hz)
                : 0;
            uint32_t fthb_us = bench_freq_hz
                ? (uint32_t)(((bench_thumb_mode_ticks - thbm_at_start) * 1000000ULL) / bench_freq_hz)
                : 0;
            uint32_t facs = bench_arm_cond_skip         - acs_at_start;
            uint32_t fass = bench_arm_ss_inst           - ass_at_start;
            uint32_t fabi = bench_arm_block_inst        - abi_at_start;
            uint32_t fabd = bench_arm_block_decodes     - abd_at_start;
            uint32_t ftss = bench_thumb_ss_inst         - tss_at_start;
            uint32_t ftbi = bench_thumb_block_inst      - tbi_at_start;
            uint32_t ftbd = bench_thumb_block_decodes   - tbd_at_start;
            n = safe_append(buf, sizeof(buf), n,
                "FRAME breakdown:\n"
                "  arm_exec=%lu us  render=%lu us  dupdate=%lu us\n"
                "  arm_mode=%lu us  thumb_mode=%lu us\n"
                "  arm_ss=%lu  arm_blk=%lu (decodes=%lu)  cond_skip=%lu\n"
                "  thumb_ss=%lu  thumb_blk=%lu (decodes=%lu)\n"
                "  slow_read=%lu  slow_write=%lu  chunk_miss=%lu\n",
                (unsigned long)fa_us, (unsigned long)fr_us,
                (unsigned long)fd_us,
                (unsigned long)farm_us, (unsigned long)fthb_us,
                (unsigned long)fass, (unsigned long)fabi, (unsigned long)fabd,
                (unsigned long)facs,
                (unsigned long)ftss, (unsigned long)ftbi, (unsigned long)ftbd,
                (unsigned long)fslr, (unsigned long)fslw,
                (unsigned long)fcm);

            // Cumulative-since-prev-scheduled-snapshot — only meaningful and
            // only reset on scheduled snapshots. Spike snapshots leave the
            // cumulative accumulators alone so the next scheduled snapshot
            // still reports a clean delta.
            if (is_scheduled) {
                static uint32_t prev_snap_frame = 0;
                uint32_t frames_in_span = loop_frame - prev_snap_frame;
                if (frames_in_span == 0) frames_in_span = 1;
                prev_snap_frame = loop_frame;

                uint32_t arm_us = bench_freq_hz
                    ? (uint32_t)((bench_arm_exec_ticks * 1000000ULL) / bench_freq_hz)
                    : 0;
                uint32_t ren_us = bench_freq_hz
                    ? (uint32_t)((bench_render_ticks * 1000000ULL) / bench_freq_hz)
                    : 0;
                uint32_t dup_us = bench_freq_hz
                    ? (uint32_t)((bench_dupdate_ticks * 1000000ULL) / bench_freq_hz)
                    : 0;
                uint32_t armm_us = bench_freq_hz
                    ? (uint32_t)((bench_arm_mode_ticks * 1000000ULL) / bench_freq_hz)
                    : 0;
                uint32_t thbm_us = bench_freq_hz
                    ? (uint32_t)((bench_thumb_mode_ticks * 1000000ULL) / bench_freq_hz)
                    : 0;
                n = safe_append(buf, sizeof(buf), n,
                    "BENCH (totals over %lu frames, freq=%lu Hz):\n"
                    "  arm_exec=%lu us (%lu/frame)\n"
                    "  render  =%lu us (%lu/frame)\n"
                    "  dupdate =%lu us (%lu/frame)\n"
                    "  arm_mode=%lu us (%lu/frame)  thumb_mode=%lu us (%lu/frame)\n"
                    "  arm_ss=%lu (%lu/frame)  arm_blk=%lu (%lu/frame)  arm_blk_decodes=%lu\n"
                    "  cond_skip=%lu (%lu/frame)\n"
                    "  thumb_ss=%lu (%lu/frame)  thumb_blk=%lu (%lu/frame)  thumb_blk_decodes=%lu\n"
                    "  arm_legacy=%lu (%lu/frame)  hist 0..F: %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu\n"
                    "  slow_read=%lu  slow_write=%lu  chunk_miss=%lu\n",
                    (unsigned long)frames_in_span,
                    (unsigned long)bench_freq_hz,
                    (unsigned long)arm_us, (unsigned long)(arm_us / frames_in_span),
                    (unsigned long)ren_us, (unsigned long)(ren_us / frames_in_span),
                    (unsigned long)dup_us, (unsigned long)(dup_us / frames_in_span),
                    (unsigned long)armm_us, (unsigned long)(armm_us / frames_in_span),
                    (unsigned long)thbm_us, (unsigned long)(thbm_us / frames_in_span),
                    (unsigned long)bench_arm_ss_inst,
                    (unsigned long)(bench_arm_ss_inst / frames_in_span),
                    (unsigned long)bench_arm_block_inst,
                    (unsigned long)(bench_arm_block_inst / frames_in_span),
                    (unsigned long)bench_arm_block_decodes,
                    (unsigned long)bench_arm_cond_skip,
                    (unsigned long)(bench_arm_cond_skip / frames_in_span),
                    (unsigned long)bench_thumb_ss_inst,
                    (unsigned long)(bench_thumb_ss_inst / frames_in_span),
                    (unsigned long)bench_thumb_block_inst,
                    (unsigned long)(bench_thumb_block_inst / frames_in_span),
                    (unsigned long)bench_thumb_block_decodes,
                    (unsigned long)bench_arm_legacy_inst,
                    (unsigned long)(bench_arm_legacy_inst / frames_in_span),
                    (unsigned long)bench_arm_legacy_hist[0x0],
                    (unsigned long)bench_arm_legacy_hist[0x1],
                    (unsigned long)bench_arm_legacy_hist[0x2],
                    (unsigned long)bench_arm_legacy_hist[0x3],
                    (unsigned long)bench_arm_legacy_hist[0x4],
                    (unsigned long)bench_arm_legacy_hist[0x5],
                    (unsigned long)bench_arm_legacy_hist[0x6],
                    (unsigned long)bench_arm_legacy_hist[0x7],
                    (unsigned long)bench_arm_legacy_hist[0x8],
                    (unsigned long)bench_arm_legacy_hist[0x9],
                    (unsigned long)bench_arm_legacy_hist[0xA],
                    (unsigned long)bench_arm_legacy_hist[0xB],
                    (unsigned long)bench_arm_legacy_hist[0xC],
                    (unsigned long)bench_arm_legacy_hist[0xD],
                    (unsigned long)bench_arm_legacy_hist[0xE],
                    (unsigned long)bench_arm_legacy_hist[0xF],
                    (unsigned long)bench_mem_slow_read,
                    (unsigned long)bench_mem_slow_write,
                    (unsigned long)bench_chunk_miss);
                // Reset cumulative for next interval.
                bench_arm_exec_ticks  = 0;
                bench_render_ticks    = 0;
                bench_dupdate_ticks   = 0;
                bench_arm_mode_ticks  = 0;
                bench_thumb_mode_ticks = 0;
                bench_mem_slow_read   = 0;
                bench_mem_slow_write  = 0;
                bench_chunk_miss      = 0;
                bench_arm_cond_skip   = 0;
                bench_arm_ss_inst     = 0;
                bench_arm_block_inst  = 0;
                bench_arm_block_decodes = 0;
                bench_thumb_ss_inst   = 0;
                bench_thumb_block_inst = 0;
                bench_thumb_block_decodes = 0;
                bench_arm_legacy_inst = 0;
                for (int hi = 0; hi < 16; hi++) bench_arm_legacy_hist[hi] = 0;
            }
            n = safe_append(buf, sizeof(buf), n, "\n");

            FILE *log = fopen("fxgba_log.txt", "a");
            if (log) {
                fwrite(buf, 1, n, log);
                fclose(log);
                if (is_scheduled) dumps_done++;
                if (is_spike) {
                    spike_dumps++;
                    last_spike_frame = loop_frame;
                }
                if (is_manual) manual_dumps++;
            }
        }
#endif // GBA_DIAG_LOG
    }

    // Final save flush on graceful exit. Always attempt the write if the
    // game ever touched the save region (covers the case where the player
    // saved in-game less than SAVE_QUIESCENT_FRAMES before exiting).
    if (have_save_path && (save_dirty || save_pending)) {
        save_write(save_path);
    }

    // Clean up ROM buffer resources
    rom_buffer_cleanup(&rom_buffer);
    thumb_block_uninit();

    gint_gba_uninit();
    arm_uninit();

    return 1;
}