// CPU test build (CPU_TEST): fixed worst-case scenarios with the load per part printed over USB;
// with FUZZ_TEST random control changes instead. Compiled to nothing in the normal firmware.
#include "app.h"
#include "cpu_test.h"
#include "debug_trace.h"
#include <cmath>

using namespace daisy;

#ifdef CPU_TEST
CpuLoadMeter cpu;
volatile bool prof_reset = true;
uint64_t prof_sum[kProfCount];
uint32_t prof_max[kProfCount], prof_blocks;

// CPU stress test, build with: C_DEFS=-DCPU_TEST make -C template. Never part of the normal
// firmware. Fills the tape with a loud test signal (a 10 s recording plus overdub layers), then
// cycles through worst-case scenarios with no controls touched and prints the audio callback
// load (average and peak) over USB serial.
struct CpuScenario {
    const char* name;
    int   mode;
    float battery, aging, type, fade, a, b;
    bool  play, overdub, moves;
    bool  live;   // live input through the tape (forced on; rows without it: off)
    bool  shock;  // window shock (P08) at full severity (rows without it: off)
    bool  fx;     // every pad effect at full: shock, crush, beat repeat /32, glitch (rows without it: off)
};
static constexpr int kFwd = GrainPlayer::FORWARD, kBwd = GrainPlayer::BACKWARD, kPing = GrainPlayer::PINGPONG;
static const CpuScenario kScenarios[] = {
    // A short loop (0.5 s) with the longest crossfade keeps half of every pass in the crossfade.
    {"stopped",                       kFwd, 0.5f, 0.0f, 0.33f,  200.0f, 0.0f, 1.0f,  false, false, false},
    {"play",                          kFwd, 0.5f, 0.0f, 0.33f,  200.0f, 0.0f, 1.0f,  true,  false, false},
    {"overdub, clean",                kFwd, 0.5f, 0.0f, 0.33f,  200.0f, 0.0f, 1.0f,  true,  true,  false},
    {"overdub, aging max, fastest",   kFwd, 1.0f, 1.0f, 1.0f,  2000.0f, 0.3f, 0.374f, true,  true,  false},
    {"  same, backward",              kBwd, 1.0f, 1.0f, 1.0f,  2000.0f, 0.3f, 0.374f, true,  true,  false},
    {"  same, ping-pong",             kPing, 1.0f, 1.0f, 1.0f,  2000.0f, 0.3f, 0.374f, true,  true,  false},
    {"  same, slowest",               kFwd, 0.0f, 1.0f, 1.0f,  2000.0f, 0.3f, 0.374f, true,  true,  false},
    {"  same, fastest, loop moving",  kFwd, 1.0f, 1.0f, 1.0f,  2000.0f, 0.3f, 0.374f, true,  true,  true},
    // The shortest loop (100 ms): the crossfade is half the loop and the loop is never held.
    {"short 100 ms, aging max, fast", kFwd, 1.0f, 1.0f, 1.0f,  2000.0f, 0.3f, 0.0f,  true,  true,  false},
    {"  same, loop moving",           kFwd, 1.0f, 1.0f, 1.0f,  2000.0f, 0.3f, 0.0f,  true,  true,  true},
    {"  same, ping-pong, moving",     kPing, 1.0f, 1.0f, 1.0f, 2000.0f, 0.3f, 0.0f,  true,  true,  true},
    {"live input, fastest, dictaphone", kFwd, 1.0f, 1.0f, 1.0f, 2000.0f, 0.3f, 0.374f, false, false, false, true},
    {"short p-p moving + shock",      kPing, 1.0f, 1.0f, 1.0f, 2000.0f, 0.3f, 0.0f,  true,  true,  true, false, true},
    {"short p-p moving + all fx",     kPing, 1.0f, 1.0f, 1.0f, 2000.0f, 0.3f, 0.0f,  true,  true,  true, false, true, true},
};

static void PrintLoad(const char* label) {
    const int avg = (int)(cpu.GetAvgCpuLoad() * 1000.0f + 0.5f);   // tenths of a percent
    const int max = (int)(cpu.GetMaxCpuLoad() * 1000.0f + 0.5f);
    hw.PrintLine("%-30s avg %3d.%d %%   peak %3d.%d %%", label, avg / 10, avg % 10, max / 10, max % 10);
    // Per part, in tenths of a percent of the block's time budget: average / peak block.
    const float budget = (float)SystemCoreClock * (float)hw.AudioBlockSize() / hw.AudioSampleRate();
    const uint32_t blocks = prof_blocks ? prof_blocks : 1;
    int a[kProfCount], m[kProfCount];
    for (int p = 0; p < kProfCount; p++) {
        a[p] = (int)(1000.0f * (float)prof_sum[p] / (float)blocks / budget + 0.5f);
        m[p] = (int)(1000.0f * (float)prof_max[p] / budget + 0.5f);
    }
    hw.PrintLine("    grains %d.%d/%d.%d  head %d.%d/%d.%d  tape %d.%d/%d.%d  block %d.%d/%d.%d  (avg/peak %%)",
                 a[0] / 10, a[0] % 10, m[0] / 10, m[0] % 10, a[1] / 10, a[1] % 10, m[1] / 10, m[1] % 10,
                 a[2] / 10, a[2] % 10, m[2] / 10, m[2] % 10, a[3] / 10, a[3] % 10, m[3] / 10, m[3] % 10);
}

void CpuTestFill() {
    // Loud material (above the soft-limit knee) on both tracks and in the lead-ins.
    uint32_t seed = 1;
    const size_t len = 48000 * 10;
    for (size_t i = 0; i < len; i++) {
        seed = seed * 1664525u + 1013904223u;
        const float noise = (float)(int32_t)seed * (1.0f / 2147483648.0f);
        const float s = 0.6f * sinf(0.0287f * (float)i) + 0.3f * noise;
        const float o = 0.5f * sinf(0.0491f * (float)i) + 0.2f * noise;
        tape[i] = {s, s, o, o};
    }
    for (size_t i = 0; i < kPreRoll; i++) pre_l[0][i] = pre_r[0][i] = 0.5f * sinf(0.0287f * (float)i);
    for (size_t i = 0; i < kOvdPre; i++) ovd_pre_l[i] = ovd_pre_r[i] = 0.3f * sinf(0.0491f * (float)i);
    player.SetPreRoll(pre_l[0], pre_r[0], kPreRoll, 0);
    head.SetPreRoll(pre_l[0], pre_r[0], kPreRoll, 0);
    pre_live = 1;
    length = len;
    player.SetLength(len);
    player.Reset();
}

void CpuTestRun() {
#ifdef DEBUG_TRACE
    // With the diagnostic trace: the log was started in main; a hang restarts the device (4 s:
    // the pause between cycles is 2 s) and the report is printed at the next cycle.
    TraceWatchdogStart(4000);
#else
    hw.StartLog(false);
#endif
    cpu.Init(hw.AudioSampleRate(), hw.AudioBlockSize());
    // Cycle counter for the per-part breakdown.
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55;   // unlock (Cortex-M7)
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    for (int cycle = 1;; cycle++) {
        System::Delay(2000);
        hw.PrintLine("CPU test, cycle %d (32-sample blocks at 48 kHz)", cycle);
#ifdef DEBUG_TRACE
        TracePrint();   // the report from before the last reset, if any
#endif
        for (const CpuScenario& s : kScenarios) {
            mode = s.mode;
            battery = s.battery;
            SetAgingFrom(s.aging, s.type);
            tape_type = s.type;
            fade_ms = s.fade;
            range_a = s.a;
            range_b = s.b;
            want_play = s.play;
            want_overdub = s.overdub;
            want_live = s.live;
            shock_sev = (s.shock || s.fx) ? 1.0f : 0.0f;
            crush_amt = s.fx ? 1.0f : 0.0f;
            rep_on = s.fx;
            rep_div = 32.0f;
            glitch_amt = s.fx ? 1.0f : 0.0f;
            phaser_p = s.fx ? 1.0f : 0.0f;
            const uint32_t t0 = System::GetNow();
            bool reset = false;
            while (System::GetNow() - t0 < 5000) {
                if (!reset && System::GetNow() - t0 >= 1000) {   // 1 s to settle
                    cpu.Reset();
                    prof_reset = true;
                    reset = true;
                }
                if (s.moves) range_a = ((System::GetNow() / 400) % 2) ? 0.32f : 0.3f;
#ifdef DEBUG_TRACE
                IWDG1->KR = 0xAAAA;   // watchdog refresh
                g_trace.main_loops = g_trace.main_loops + 1;
#endif
                System::Delay(5);
            }
            PrintLoad(s.name);
        }
    }
}

#ifdef FUZZ_TEST
// Hardware fuzz test, with C_DEFS="-DCPU_TEST -DDEBUG_TRACE -DFUZZ_TEST": instead of the fixed
// scenarios, a random change to the controls every 50-200 ms, forever (pads by the same rules as
// the real pad handler, so only reachable states). Heartbeat + trace report every 10 s.
static uint32_t fuzz_seed = 0x2468ACE1u;
static float FuzzRand() {   // 0 .. 1
    fuzz_seed ^= fuzz_seed << 13;
    fuzz_seed ^= fuzz_seed >> 17;
    fuzz_seed ^= fuzz_seed << 5;
    return (float)(fuzz_seed >> 8) * (1.0f / 16777216.0f);
}
static float FuzzPress() { return (FuzzRand() < 0.5f) ? 0.0f : FuzzRand(); }   // released half the time

void CpuFuzzRun() {
#ifdef DEBUG_TRACE
    TraceWatchdogStart(4000);
#else
    hw.StartLog(false);
#endif
    cpu.Init(hw.AudioSampleRate(), hw.AudioBlockSize());
    want_play = true;
    uint32_t next = 0, beat = System::GetNow(), changes = 0;
    while (1) {
#ifdef DEBUG_TRACE
        IWDG1->KR = 0xAAAA;   // watchdog refresh
        g_trace.main_loops = g_trace.main_loops + 1;
#endif
        const uint32_t now = System::GetNow();
        if (now >= next) {
            next = now + 50 + (uint32_t)(FuzzRand() * 150.0f);
            changes++;
            const float r = FuzzRand();
            if (r < 0.08f) {
                mode = (int)(FuzzRand() * 2.999f);
            } else if (r < 0.30f) {
                range_a = FuzzRand();
            } else if (r < 0.50f) {
                range_b = (FuzzRand() < 0.6f) ? 0.15f * FuzzRand() : FuzzRand();   // often the shortest loops
            } else if (r < 0.56f) {
                battery = FuzzRand();
            } else if (r < 0.60f) {
                fade_ms = 10.0f + 1990.0f * FuzzRand();
            } else if (r < 0.64f) {
                const float type = FuzzRand();
                SetAgingFrom(FuzzRand(), type);
                tape_type = type;
            } else if (r < 0.71f) {   // P11, as the pad handler
                if (want_overdub) want_overdub = false;
                else if (want_live) want_live = false;
                else if (!want_record && length > 0) want_overdub = true;
                else if (!want_record) want_live = true;
            } else if (r < 0.73f) {   // P10, as the pad handler
                if (want_record) want_record = false;
                else { want_overdub = false; want_live = false; want_record = true; }
            } else if (r < 0.76f) {   // P03
                want_play = !want_play;
            } else if (r < 0.80f) {
                friction = 2.0f * FuzzPress() * FuzzPress() * FuzzPress();
            } else if (r < 0.84f) {
                shock_sev = FuzzPress();
            } else if (r < 0.88f) {
                crush_amt = FuzzPress();
            } else if (r < 0.92f) {
                rep_on = FuzzRand() < 0.5f;
                rep_div = (float)(4 << (int)(FuzzRand() * 3.999f));
            } else if (r < 0.96f) {
                glitch_amt = FuzzPress();
            } else {
                phaser_p = FuzzPress();
            }
        }
        if (now - beat >= 10000) {
            beat = now;
            const int avg = (int)(cpu.GetAvgCpuLoad() * 100.0f + 0.5f);
            const int max = (int)(cpu.GetMaxCpuLoad() * 100.0f + 0.5f);
            hw.PrintLine("fuzz: %lu changes, load avg %d %% peak %d %%, length %lu, mode %d, play %d od %d live %d rec %d",
                         (unsigned long)changes, avg, max, (unsigned long)length, (int)mode,
                         (int)want_play, (int)want_overdub, (int)want_live, (int)want_record);
            cpu.Reset();
#ifdef DEBUG_TRACE
            TracePrint();
#endif
        }
        System::Delay(2);
    }
}
#endif
#endif
