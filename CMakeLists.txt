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
  src/arm_mem.c
  src/arm.c
  src/dma.c
  src/extram.c
  src/gint_gba.c
  src/io.c
  src/rom_buffer.c
  src/sound.c
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
    src/arm_mem.c
    src/rom_buffer.c
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
