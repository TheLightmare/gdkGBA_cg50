#!/usr/bin/env bash
# Build wrapper for gdkGBA_cg50 (fxCG50).
#
# Usage:
#   ./build.sh                   # release (default)
#   ./build.sh release           # fast, no diagnostics
#   ./build.sh bench             # phase timers + slow-path counters
#   ./build.sh trace             # bench + on-screen overlay + fxgba_log.txt
#   ./build.sh clean             # wipe build-cg/
#
# Extra args after the flavor go to cmake, e.g.
#   ./build.sh bench -DGBA_FXLINK=ON
#
# Why this script exists: `fxsdk build-cg` doesn't forward -D args to the
# cmake configure step (it forwards them to make, where they crash). So we
# let fxsdk bootstrap the toolchain on the first run, then drive cmake
# directly afterward to actually select the flavor.
#
# Output: build-cg/fxgba.g3a (also copied to ./fxgba.g3a).

set -e
cd "$(dirname "$0")"

flavor=${1:-release}
shift || true

case "$flavor" in
    release|bench|trace)
        if [[ ! -f build-cg/CMakeCache.txt ]]; then
            echo ">>> bootstrapping build-cg/ via fxsdk (one-time)"
            fxsdk build-cg || true
            if [[ ! -f build-cg/CMakeCache.txt ]]; then
                echo "fxsdk bootstrap failed" >&2
                exit 1
            fi
        fi

        echo ">>> configuring (GBA_BUILD=$flavor)"
        cmake -B build-cg -DGBA_BUILD="$flavor" "$@"

        echo ">>> building"
        cmake --build build-cg
        ;;
    clean)
        rm -rf build-cg
        echo "removed build-cg/"
        ;;
    -h|--help|help)
        sed -n '2,17p' "$0"
        ;;
    *)
        echo "unknown flavor: $flavor" >&2
        echo "usage: ./build.sh {release|bench|trace|clean} [cmake args...]" >&2
        exit 2
        ;;
esac
