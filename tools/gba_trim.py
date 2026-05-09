#!/usr/bin/env python3
# Strip trailing 0xFF padding from a GBA ROM. Most cart dumps are padded to
# the physical chip size (16/32 MB) but the game itself is much smaller.
# Trimmed ROMs work transparently in the emulator because cart reads past
# rom_size return 0 (or are masked by rom_mask), and Nintendo-logo / header
# checks only touch the first 0xC0 bytes.
import sys

if len(sys.argv) != 3:
    print(f"usage: {sys.argv[0]} input.gba output.gba")
    sys.exit(1)

with open(sys.argv[1], "rb") as f:
    data = f.read()

# Strip trailing 0xFF bytes (most common cart padding).
trimmed = data.rstrip(b"\xff")

# Round up to the next 4-byte boundary so 32-bit reads at the tail stay
# aligned (the emulator's rom_buffer pads to chunk size with zeros anyway,
# but this avoids any short-read edge cases).
pad = (-len(trimmed)) & 3
trimmed += b"\x00" * pad

with open(sys.argv[2], "wb") as f:
    f.write(trimmed)

print(f"  in: {len(data):>10,} bytes")
print(f" out: {len(trimmed):>10,} bytes  ({100*len(trimmed)/len(data):.1f}%)")
print(f"saved: {len(data)-len(trimmed):>10,} bytes")
