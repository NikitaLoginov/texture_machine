#pragma once
#include <cstdint>

// Diagnostic build only (build with C_DEFS=-DDEBUG_TRACE, see main.cpp): the audio path leaves
// breadcrumbs in backup SRAM, which survives a reset. A fault or a watchdog reset (hang) restarts
// the device, which then prints the last breadcrumbs over USB serial. In the normal build the
// macros are empty.
#ifdef DEBUG_TRACE
enum TraceStage : uint32_t {
    kTrBlock = 1,    // callback start
    kTrPlayer,       // GrainPlayer::Process
    kTrGrainStart,   // a grain starts (loop change)
    kTrRead,         // grain reads
    kTrHead,         // RecordHead::Process
    kTrGhost,        // ghost writes after a jump
    kTrWrite,        // RecordHead::Write
    kTrWobble,       // printed stutter
    kTrLead,         // lead zone write
    kTrTape,         // tape sound, tilt, output
    kTrBlockEnd,     // callback done
};

struct DebugTrace {
    uint32_t magic;       // kTraceArmed while the firmware runs
    uint32_t cause;       // 1 = fault (set by the fault handler)
    uint32_t blocks;      // audio callbacks so far
    uint32_t main_loops;  // main loop passes so far
    uint32_t stage;       // TraceStage the audio path is in
    // Fault registers and where it happened.
    uint32_t pc, lr, psr, cfsr, hfsr, bfar, mmfar;
    // Player.
    int32_t  mode, rs, rl;
    float    pos, step, speed;
    // Record head.
    int32_t  h_rs, h_rl, count, k, wi, ghost_left, fade_left;
    float    h_pos, h_step, from, ghost_from;
};

extern volatile DebugTrace g_trace;
#define TRACE_STAGE(s) (g_trace.stage = (s))
#define TRACE_SET(field, v) (g_trace.field = (v))
#else
#define TRACE_STAGE(s) ((void)0)
#define TRACE_SET(field, v) ((void)0)
#endif
