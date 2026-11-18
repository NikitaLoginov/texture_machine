#pragma once
// CPU test build (CPU_TEST, optionally with FUZZ_TEST): the audio callback's load meter and
// per-part cycle counts, and the test runs (cpu_test.cpp). Without CPU_TEST the PROF macros are
// empty and nothing here exists.
#ifdef CPU_TEST
#include "daisy_seed.h"
#include <cstdint>

extern daisy::CpuLoadMeter cpu;   // audio callback load
// Cycles spent per part of the callback, per block (cycle counter), for the breakdown.
enum { kProfGrains, kProfHead, kProfTape, kProfBlock, kProfCount };
extern volatile bool prof_reset;   // set by the test loop, cleared by the callback
extern uint64_t prof_sum[kProfCount];
extern uint32_t prof_max[kProfCount], prof_blocks;
#define PROF_BEGIN(t) const uint32_t t = DWT->CYCCNT
#define PROF_END(part, t) prof_now[part] += DWT->CYCCNT - t

void CpuTestFill();   // fills the tape with a loud test signal
void CpuTestRun();    // the fixed scenarios, forever
void CpuFuzzRun();    // FUZZ_TEST: random control changes, forever
#else
#define PROF_BEGIN(t)
#define PROF_END(part, t)
#endif
