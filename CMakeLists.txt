# Configure with [fxsdk build-fx] or [fxsdk build-cg], which provide the
# toolchain file and module path of the fxSDK

cmake_minimum_required(VERSION 3.15)
project(MyAddin)

include(GenerateG1A)
include(GenerateG3A)
include(Fxconv)
find_package(Gint 2.9 REQUIRED)

set(SOURCES
  src/main.c
  src/arm_block.c
  src/arm_mem.c
  src/arm.c
  src/bench.c
  src/dma.c
  src/extram.c
  src/gint_gba.c
  src/io.c
  src/rom_buffer.c
  src/save_file.c
  src/sound.c
  src/thumb_block.c
  src/timer.c
  src/video.c
  # ...
)



# Shared assets, fx-9860G-only assets and fx-CG-50-only assets
set(ASSETS
  # ...
)
set(ASSETS_fx
  assets-fx/example.png
  # ...
)
set(ASSETS_cg
  assets-cg/example.png
  # ...
)

fxconv_declare_assets(${ASSETS} ${ASSETS_fx} ${ASSETS_cg} WITH_METADATA)

add_executable(myaddin ${SOURCES} ${ASSETS} ${ASSETS_${FXSDK_PLATFORM}})
target_compile_options(myaddin PRIVATE -Wall -Wextra -Os)
target_link_libraries(myaddin Gint::Gint)

# Build flavor: release | bench | trace
#
# release (default): no diagnostics. The interpreter, memory dispatch and
#                    renderer run without any bench timer wraps, slow-path
#                    counters, on-screen overlay, fxgba_log.txt snapshots,
#                    or arm trace bookkeeping. Use for performance numbers.
# bench:             enables phase timers (bench_arm_exec_ticks etc.) and
#                    slow-path counters (bench_mem_slow_*, bench_chunk_miss).
#                    No overlay, no log file. Adds bench_now()/bench_elapsed
#                    overhead around arm_exec / render_line / dupdate.
# trace:             everything bench has, plus the on-screen heartbeat,
#                    boot dump + per-frame snapshot machinery to
#                    fxgba_log.txt, PC trace, undef/SWI counts. Behaves
#                    like the legacy build.
#
# Power users can also set GBA_BENCH / GBA_DEBUG_OVERLAY / GBA_DIAG_LOG /
# GBA_TRACE individually -- GBA_BUILD just expands to a sensible default
# combination. See src/build_flags.h.
set(GBA_BUILD "release" CACHE STRING "Build flavor: release | bench | trace")
set_property(CACHE GBA_BUILD PROPERTY STRINGS release bench trace)

if(GBA_BUILD STREQUAL "trace")
    target_compile_definitions(myaddin PRIVATE
        GBA_BENCH=1
        GBA_DEBUG_OVERLAY=1
        GBA_DIAG_LOG=1
        GBA_TRACE=1)
elseif(GBA_BUILD STREQUAL "bench")
    target_compile_definitions(myaddin PRIVATE GBA_BENCH=1)
elseif(GBA_BUILD STREQUAL "release")
    # No diagnostic flags. The hot path stays clean.
else()
    message(FATAL_ERROR
        "GBA_BUILD must be one of: release | bench | trace (got '${GBA_BUILD}')")
endif()
message(STATUS "GBA_BUILD = ${GBA_BUILD}")

# Optional: USB bulk streaming to fxlink for live recording. Adds a
# non-trivial chunk of gint USB code to the binary and pulls the addin into
# a usb_open() at startup, so leave it off for normal release builds.
# Enable with `cmake -DGBA_FXLINK=ON ...` (or `fxsdk build-cg -DGBA_FXLINK=ON`).
option(GBA_FXLINK "Stream emulator frames to fxlink over USB" OFF)
if(GBA_FXLINK)
    target_compile_definitions(myaddin PRIVATE GBA_FXLINK=1)
endif()
message(STATUS "GBA_FXLINK = ${GBA_FXLINK}")

# Link-time optimization: lets the linker inline across translation
# units. Critical here because arm_read() in arm_mem.c calls
# rom_buffer_read_32() in rom_buffer.c -- without -flto the call goes
# through a real function call every memory access. With -flto GCC can
# inline rom_buffer_read_32's hot path (cache-hit + byte assembly)
# directly into arm_read, saving the call overhead per LDR/STR.
target_compile_options(myaddin PRIVATE -flto)
target_link_options(myaddin PRIVATE -flto)

# Hot files: ARM/Thumb interpreter, ROM fetch, IO bus, renderer, DMA
# and timer (called per scanline / per memory access). Replace the
# project-wide -Os with -O2 (or -O3 for the interpreter dispatch where
# inlining the per-opcode functions pays off the most). -fno-strict-aliasing
# defangs the type-punning rules; the original gdkGBA and our union-based
# io_reg / arm_word usage rely on punning.
set_source_files_properties(
    src/arm.c
    PROPERTIES COMPILE_FLAGS "-O3 -fno-strict-aliasing"
)
set_source_files_properties(
    src/arm_block.c
    src/arm_mem.c
    src/rom_buffer.c
    src/thumb_block.c
    src/video.c
    src/io.c
    src/dma.c
    src/timer.c
    PROPERTIES COMPILE_FLAGS "-O2 -fno-strict-aliasing"
)

if("${FXSDK_PLATFORM_LONG}" STREQUAL fx9860G)
  generate_g1a(TARGET myaddin OUTPUT "fxgba.g1a"
    NAME "fxgba" ICON assets-fx/icon.png)
elseif("${FXSDK_PLATFORM_LONG}" STREQUAL fxCG50)
  generate_g3a(TARGET myaddin OUTPUT "fxgba.g3a"
    NAME "fxgba" ICONS assets-cg/icon-uns.png assets-cg/icon-sel.png)
endif()
