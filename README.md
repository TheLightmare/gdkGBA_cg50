# gdkGBA_cg50

A port of [gdkChan's gdkGBA](https://github.com/gdkchan/gdkGBA) Game Boy
Advance emulator to the **Casio fx-CG50** graphing calculator, built on
[fxSDK](https://gitea.planet-casio.com/Lephenixnoir/fxsdk) and
[gint](https://gitea.planet-casio.com/Lephenixnoir/gint).

## Status

**Cart-runnable.** Real games boot, render their title screens, take
button input, and play. As of the latest changes:

- ✅ Boots through BIOS / cart startup
- ✅ Mario Kart Super Circuit reaches its title screen, takes input
- ✅ Zelda Minish Cap runs through title, save select, and gameplay
- ✅ Real game graphics (BG layers, OBJ, palette) render correctly
- ⚠️ Performance: ~10-12 fps steady, ~7 fps in heavy gameplay scenes
  with frameskip=1 (target: 60 fps real GBA). **Ptune4 F5 preset is
  effectively required** — the SH4A is bus-bound, so CPU-only overclocks
  (Ptune3) don't help; the F5 preset's bus overclock does.
- ❌ Audio not supported (it eats through performance, so i probably will not support it at all)
- ❌ Save persistence (EEPROM/SRAM/Flash) lost on add-in exit
- ❌ Some games with full 256 KB EWRAM may corrupt without OS ≤ 03.06

## Build

You need fxSDK and gint installed. From the project root:

```bash
fxsdk build-cg
```

Produces `fxgba.g3a` in the project root. Transfer it to the calculator's
storage memory along with:

- `gba_bios.bin` — a GBA BIOS file (real Nintendo BIOS or a homebrew
  replacement like Normmatt's). Must be exactly 16 KB.
- A `.gba` ROM file. The add-in scans `\\fls0\*.gba` and loads the first
  match (alphabetical); falls back to `test.gba` if none is found.

## Controls

| GBA button | Calculator key |
|---|---|
| D-pad | Arrows |
| A | SHIFT |
| B | ALPHA |
| L (trigger) | F1 |
| R (trigger) | F6 |
| Select | OPTN |
| Start | EXE |
| Quit emulator | MENU or EXIT |

## Tools

- `tools/gba_trim.py <input.gba> <output.gba>` — strip trailing `0xFF`
  padding from cart dumps. Most ROMs are padded to the full physical
  cart size (16/32 MB) but only a fraction is real game data. Trimmed
  ROMs work transparently in the emulator and free up calculator
  storage.


Highlights of the development:

- **Pass 1** — structural fixes (NULL `screen` pointer, RGB565
  conversion, ROM buffer optimisation, input rework, build-system gaps)
- **Pass 3** — Thumb decoder bug: `t16_blx`/`t16_blx_h1` (ARMv5+
  instructions) were registered for the GBA's ARMv4T, sending undefined
  opcodes through bogus BLX paths
- **Pass 4** — endianness: SH4 is big-endian; the original LE-host code
  had bugs in union field ordering (`io_reg`, `arm_word`) and in the
  `arm_fetch` path's native multi-byte casts
- **Pass 5** — EWRAM buffer overflow (`wram_board[0x10000]` masked with
  `0x3FFFF`) was corrupting the gint heap
- **Pass 6** — replacement BIOS quirks; HLE'd common SWI calls
  (`CpuSet`, `CpuFastSet`, `Div`, `Halt`, `RegisterRamReset`,
  `IntrWait`, `VBlankIntrWait`)
- **Pass 7** — SH4 alignment fault in `eeprom_write` from an unaligned
  `*(uint64_t *)` cast
- **Pass 8** — filesystem world-switch: `fread` from gint world panicked
  on fragmented files; the fix wraps chunk loads in
  `gint_world_switch`. This also rules out `_ostk` for EWRAM since the
  OS stack becomes live during world switches.
- **Pass 9 — performance**. Diagnosed the bottleneck via instrumented
  per-phase wall-clock timing (TMU-backed bench in [`src/bench.c`](src/bench.c))
  and the empirical observation that Ptune3 (CPU-only overclock) gave
  zero gain while Ptune4 F5 (CPU + bus overclock) gave a clear gain →
  **bus-bound, not CPU-bound**. Optimization moves were then ordered by
  what cuts main-RAM bus traffic, not by what cuts host instructions:
    - **Lazy NZCV flags** ([`src/arm.c`](src/arm.c)). Replaced four
      RMW-on-`cpsr` per arithmetic op with four single-store register
      writes; only sync to `cpsr` at MRS/MSR/SPSR boundaries.
      ~3-5× speedup of the interpreter alone.
    - **Page-table memory dispatch**. `mem_read_pages[256]` and
      `mem_write_pages[256]` indexed by `addr >> 24` replace the
      switch-and-helper-call dispatch in `arm_readb` / `arm_writeb` /
      etc. Hot regions (EWRAM/IWRAM/PRAM/OAM) become a 3-instruction
      fast path; VRAM stays inline because of its mirroring quirk.
    - **Inline Thumb fast paths**. The 4 most common Thumb opcodes
      (MOV/CMP/ADD/SUB imm8) inlined into `t16_step` via a switch on
      top-5 bits, with their helpers (`t16_data_imm8_op`,
      `arm_arith_*`) marked `static inline` so the compiler folds the
      whole chain into the dispatcher.
    - **PRAM write fast path** inlines the BGR555→RGB565 conversion
      directly in `arm_writeh`/`arm_write` for region 0x05, eliminating
      per-byte `pram_write` calls during palette fade animations.
    - **On-chip RAM placement** of hot data (`arm_r` register file,
      lazy flags, dispatch state, `thumb_proc[2048]`, page tables,
      `rom_buffer`) into XYRAM (16 KB, 1-cycle access, never
      cache-evicted). Configured via `gint_set_onchip_save_mode(BACKUP)`
      with a 20 KB buffer so state survives `gint_world_switch`. ~12.8 KB
      of XYRAM used out of 16 KB.
    - **Inline ROM chunk-cache hot path** in `arm_fetch` /
      `arm_fetchh`. The `rom_buffer.last_chunk_*` fields are checked
      directly inline; with `rom_buffer` now in XYRAM, the lookup is
      0-bus-cost. Single biggest win: ~40 % off `arm_exec` in steady
      state (~280 K fetches/frame, each previously paying for the
      `rom_buffer_read_*` function call + cache-missed lookup state).
    - **Smaller, more numerous ROM chunks** (32 KB × 88 = 2.75 MB
      cache, vs. previous 64 KB × 32 = 2 MB). For a 14 MB cart whose
      working set is "spread thin", finer granularity reduces thrashing
      materially: chunk_miss/frame dropped from ~4 to ~1.
    - **Tile-coherent renderer** in [`src/video.c`](src/video.c).
      Per-pixel BG and OBJ work was recomputing tilemap entries, tile
      base addresses, and palette flags 8× per tile. Restructured to
      walk tile-by-tile, doing setup once per 8 pixels. ~46 % off
      render time in steady state.
    - **Net result**: roughly **3 fps → 12 fps** steady, **~7 fps**
      in heavy gameplay scenes. Bench breakdown at snap-4 steady-state:
      arm_exec 60 ms, render 14 ms, dupdate 5.5 ms = ~80 ms/frame.

A French translation of the log is available at
[`docs/PORT_LOG.fr.md`](docs/PORT_LOG.fr.md).

## Memory

Memory budget on the CG50 is tight. The emulator uses:

- **256 KB EWRAM** (full GBA spec) when running on **OS ≤ 03.06**, via
  the `extram` arena at `0x8c200000`. On newer OSes this region is
  unavailable and we fall back to a 64 KB clamped EWRAM with mirroring
  (some games will misbehave).
- **88 × 32 KB ROM chunks** (2.75 MB total) cached on demand from the
  cart. Smaller chunks → finer cache granularity, which matters for the
  spread-thin access patterns of large carts (Zelda Minish Cap is ~14 MB
  trimmed). The chunk allocator prefers `extram` when available; falls
  back to `_uram` and degrades gracefully to fewer chunks if memory is
  short.
- **~12.8 KB of XYRAM** (16 KB on-chip 1-cycle SRAM) holds hot
  interpreter data: ARM register file, lazy NZCV flags, dispatch state,
  `thumb_proc[2048]` table, memory page tables, and the
  `rom_buffer` chunk-cache state. Backed up across `gint_world_switch`
  via a 20 KB buffer.
- **64 KB SRAM** + **8 KB EEPROM** + **128 KB Flash** for save data
  (Flash is lazily allocated on first cart access).

The boot screen reports the actual EWRAM size and ROM-chunk count so
you can see what the emulator decided.

## Performance

Currently ~10-12 fps in steady state and ~7 fps in heavy gameplay scenes
on Zelda Minish Cap with `FRAMESKIP=1` (the default), running at Ptune4
F5 (CPU 232 MHz / Bus 58 MHz on default CG50). **The bus-bound diagnosis
matters here**: stock CG50 (Ptune off) and Ptune3 (CPU-only OC) give
similar performance; only Ptune4 F5's bus overclock helps because the
emulator spends most cycles on bus stalls, not CPU work.

The compile flags are tuned per-file: `arm.c` at `-O3`, the other hot
files (`arm_mem.c`, `rom_buffer.c`, `video.c`, `io.c`, `dma.c`,
`timer.c`) at `-O2 -fno-strict-aliasing`. Cold files keep `-Os` so the
binary doesn't bloat unnecessarily.

You can tune `FRAMESKIP` in `src/video.c`:

- `FRAMESKIP=0` — render every frame (slowest, smoothest)
- `FRAMESKIP=1` — render 1 in 2 (default)
- `FRAMESKIP=2` — render 1 in 3 (very fast, choppy)

### Built-in benchmark

[`src/bench.c`](src/bench.c) reserves a TMU (~250 ns resolution) and
the main loop captures per-phase wall time (`arm_exec`, `render`,
`dupdate`) plus event counters (`slow_read`, `slow_write`,
`chunk_miss`). Diagnostic snapshots fire at scheduled frames and on any
frame whose total time exceeds 200 ms (capped at 5 spike captures with
a 30-frame minimum gap). Output goes to `fxgba_log.txt` on the
calculator. Useful for diagnosing which scenes are interpreter-bound
vs. render-bound vs. chunk-thrashing.

## Known limitations

- Performance is far from real-time. ~15-20 % of full GBA speed at
  Ptune4 F5; lower without bus overclock.
- No audio output.
- Save data is lost on exit. Should be persisted to a sidecar file.
- Newer Casio OSes (≥ 03.07) reserve the `extram` region; on those, we
  alias EWRAM and some games corrupt their state.
- No ROM-select menu; the first `.gba` file alphabetically is used.
- Bus-bound on the SH4A means further perf gains require either
  reducing per-instruction main-RAM accesses (decoded interpreter cache,
  JIT) or running on the rare CG50 variants with faster bus.

## Credits

- Original GBA emulator: [gdkChan's gdkGBA](https://github.com/gdkchan/gdkGBA)
- gint / fxSDK: [Lephenixnoir](https://gitea.planet-casio.com/Lephenixnoir)
- `extram` arena pattern: based on [Slyvtt's Shmup](https://gitea.planet-casio.com/Slyvtt/Shmup)
- Port to fx-CG50: Lightmare

## License

The original gdkGBA is released into the public domain. This port keeps
the same status. See `src/LICENSE.txt`.
