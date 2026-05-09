# gdkGBA_cg50

A port of [gdkChan's gdkGBA](https://github.com/gdkchan/gdkGBA) Game Boy
Advance emulator to the **Casio fx-CG50** graphing calculator, built on
[fxSDK](https://gitea.planet-casio.com/Lephenixnoir/fxsdk) and
[gint](https://gitea.planet-casio.com/Lephenixnoir/gint).

## Status

**Cart-runnable.** Real games boot, render their title screens, and
take button input. As of the latest changes:

- ✅ Boots through BIOS / cart startup
- ✅ Mario Kart Super Circuit reaches its title screen, takes input
- ✅ Real game graphics (BG layers, OBJ, palette) render to the CG50 screen
- ⚠️ Performance is ~6-8 fps with frameskip=1 (target: 60 fps real GBA)
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

A French translation of the log is available at
[`docs/PORT_LOG.fr.md`](docs/PORT_LOG.fr.md).

## Memory

Memory budget on the CG50 is tight. The emulator uses:

- **256 KB EWRAM** (full GBA spec) when running on **OS ≤ 03.06**, via
  the `extram` arena at `0x8c200000`. On newer OSes this region is
  unavailable and we fall back to a 64 KB clamped EWRAM with mirroring
  (some games will misbehave).
- **8 × 64 KB ROM chunks** (512 KB total) cached on demand from the
  cart. The chunk allocator prefers `extram` when available; falls back
  to `_uram` and degrades gracefully to fewer chunks if memory is short.
- **64 KB SRAM** + **8 KB EEPROM** + **128 KB Flash** for save data
  (Flash is lazily allocated on first cart access).

The boot screen reports the actual EWRAM size and ROM-chunk count so
you can see what the emulator decided.

## Performance

Currently 6-8 fps on Mario Kart Super Circuit with `FRAMESKIP=1` (the
default). The compile flags are tuned per-file: `arm.c` at `-O3`, the
other hot files (`arm_mem.c`, `rom_buffer.c`, `video.c`, `io.c`,
`dma.c`, `timer.c`) at `-O2 -fno-strict-aliasing`. Cold files keep
`-Os` so the binary doesn't bloat unnecessarily.

You can tune `FRAMESKIP` in `src/video.c`:

- `FRAMESKIP=0` — render every frame (slowest, smoothest)
- `FRAMESKIP=1` — render 1 in 2 (default)
- `FRAMESKIP=2` — render 1 in 3 (very fast, choppy)

## Known limitations

- Performance is far from real-time. ~5-10 % of full GBA speed.
- No audio output. 
- Save data is lost on exit. Should be persisted to a sidecar file.
- Newer Casio OSes (≥ 03.07) reserve the `extram` region; on those, we
  alias EWRAM and some games corrupt their state.
- No ROM-select menu; the first `.gba` file alphabetically is used.

## Credits

- Original GBA emulator: [gdkChan's gdkGBA](https://github.com/gdkchan/gdkGBA)
- gint / fxSDK: [Lephenixnoir](https://gitea.planet-casio.com/Lephenixnoir)
- `extram` arena pattern: based on [Slyvtt's Shmup](https://gitea.planet-casio.com/Slyvtt/Shmup)
- Port to fx-CG50: Lightmare

## License

The original gdkGBA is released into the public domain. This port keeps
the same status. See `src/LICENSE.txt`.
