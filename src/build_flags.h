#ifndef BUILD_FLAGS_H
#define BUILD_FLAGS_H

// Central interpretation of the GBA_BUILD compile-time flags. CMakeLists.txt
// translates -DGBA_BUILD=release|bench|trace into the four orthogonal
// switches below; this header normalises them and exposes the helper macros
// that hot-path code uses to opt-in/out of bench instrumentation without
// scattering #ifs at every call site.
//
//   GBA_BENCH         TMU phase timers + slow-path counters.
//   GBA_DEBUG_OVERLAY Every-frame on-screen heartbeat strip in video.c.
//   GBA_DIAG_LOG      Boot dump + per-frame snapshots written to
//                     fxgba_log.txt, plus the startup getkey() screens.
//                     Implies GBA_BENCH (snapshots read bench accumulators).
//   GBA_TRACE         PC trace, undef/SWI/HLE-IRQ counts, first-low tripwire.
//                     Compatible with the legacy ARM_TRACE_ENABLE switch.

#ifndef GBA_BENCH
#  define GBA_BENCH 0
#endif
#ifndef GBA_DEBUG_OVERLAY
#  define GBA_DEBUG_OVERLAY 0
#endif
#ifndef GBA_DIAG_LOG
#  define GBA_DIAG_LOG 0
#endif
#ifndef GBA_TRACE
#  define GBA_TRACE 0
#endif

// GBA_DIAG_LOG snapshots dump bench accumulators and the arm trace state
// (PC trace, SWI counts, undef capture, first-low tripwire). Force the
// dependencies on so partial flag combinations don't link with dangling
// references to disabled state.
#if GBA_DIAG_LOG && !GBA_BENCH
#  undef GBA_BENCH
#  define GBA_BENCH 1
#endif
#if GBA_DIAG_LOG && !GBA_TRACE
#  undef GBA_TRACE
#  define GBA_TRACE 1
#endif

// Back-compat: legacy code still uses ARM_TRACE_ENABLE directly. Auto-define
// it from GBA_TRACE so the new flag is the only thing builders need to set.
#if GBA_TRACE && !defined(ARM_TRACE_ENABLE)
#  define ARM_TRACE_ENABLE 1
#endif

// Wrap a hot-path expression with a TMU phase-timer accumulate. In release
// builds this is just `expr;` -- no TMU read, no add, no extra locals.
// Callers must already include <stdint.h> and "bench.h" for the
// uint32_t / bench_now / bench_elapsed declarations.
//
//   BENCH_TIME_BLOCK(bench_arm_exec_ticks, arm_exec(CYC_LINE_HBLK0));
//
// Multiple statements need braces around `expr` at the call site.
#if GBA_BENCH
#  define BENCH_TIME_BLOCK(accum, expr)                                       \
       do {                                                                   \
           uint32_t _bench_t0 = bench_now();                                  \
           expr;                                                              \
           (accum) += bench_elapsed(_bench_t0);                               \
       } while (0)
#  define BENCH_INC(counter) ((counter)++)
#else
#  define BENCH_TIME_BLOCK(accum, expr) do { expr; } while (0)
#  define BENCH_INC(counter) ((void)0)
#endif

#endif // BUILD_FLAGS_H
