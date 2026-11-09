#include "daisy_seed.h"
#include "daisysp.h"
#include "touch/touch.h"
#include "grain_player.h"
#include "control_filter.h"
#include "motor.h"
#include "tape_format.h"
#include "record_head.h"
#include "fast_math.h"
#include "tilt_eq.h"
#include "debug_trace.h"
#include <cmath>

using namespace daisy;
using namespace daisysp;
using namespace synthux;

static DaisySeed   hw;
static Touch       touch;
static GrainPlayer player;
static Adsr env;
static TapeFormat fmt_l, fmt_r;   // tape type character per channel
static TiltEq     tilt_l, tilt_r; // S30: tone tilt on the device output
static Motor      motor;          // one tape transport for both channels
static RecordHead head;           // overdub record head, follows the playhead
static ControlFilter ctl[8];   // S30..S35, left fader (S36), right fader (S37)
static Pickup drive_pick(0.5f, true);     // S35 normal job: drive
static Pickup vol_pick(0.7071f, false);   // S35 while P01 is held: output volume (starts at unity)
static Pickup mix_pick(0.5f, false);      // S35 while P01 is held, right switch down: device / input mix

static constexpr size_t kMaxSamples = 48000 * 30;   // 30 s per channel
static float DSY_SDRAM_BSS buf_l[kMaxSamples];
static float DSY_SDRAM_BSS buf_r[kMaxSamples];
static float DSY_SDRAM_BSS ovd_l[kMaxSamples];   // overdub track, same positions as the recording
static float DSY_SDRAM_BSS ovd_r[kMaxSamples];
static constexpr size_t kOvdPre = 2048;           // overdub track lead-in before sample 0
static float ovd_pre_l[kOvdPre];
static float ovd_pre_r[kOvdPre];
static constexpr size_t kPreRoll = 48000 * 2;       // 2 s input history = lead-in of a recording
static float DSY_SDRAM_BSS pre_l[2][kPreRoll];      // two rings: one live, one frozen
static float DSY_SDRAM_BSS pre_r[2][kPreRoll];

// Written by main loop, read by audio callback
static volatile bool  want_play    = false;
static volatile bool  want_record  = false;
static volatile bool  want_overdub = false;
static volatile float battery = 0.5f;
static volatile float range_a = 0.0f, range_b = 1.0f;
static volatile int   mode = GrainPlayer::FORWARD;
static volatile float fade_ms = 200.0f, tape_type = 0.33f;
static volatile float drive = 1.0f;    // S35: multiplier on the tape saturation drive, 0.5 .. 4
static volatile float bump = 0.0f;     // S35 right of middle: head bump amount 0 .. 1
static volatile float tilt = 0.0f;     // S30: tilt EQ, -1 darker .. 0 flat .. 1 brighter
static volatile float age_coef = 1.0f, age_hiss = 0.0f, age_wob = 0.0f, age_wob_pitch = 0.0f;   // overdub aging per pass (S32)
static volatile float volume = 1.0f;   // output volume, 0 .. 2 (P01 + S35)
// Right switch = overdub mode: up = aging + volume drop per pass, center = aging only, down = aging
// only and P01 + S35 mixes the device output (0) with the dry input (1).
static volatile float keep = 0.891f;   // share of the old layers kept per pass (1 = no drop)
static volatile bool  mix_mode = false;
static volatile float mix = 0.5f;      // device / dry input mix in mix_mode, 0 .. 1

// Written by audio callback, read by main loop
static volatile size_t length = 0;   // recorded samples per channel

// Callback-only state
static bool    was_play = false, was_record = false;
static size_t  rec_pos = 0;
static int     pre_live = 0;      // ring the input is written to
static size_t  pre_pos = 0;       // next write index in the live ring (= its oldest sample)
static float   rec_gain = 0.0f;   // overdub write level 0..1, ramps on P11 on/off
static float   mon_gain = 0.0f;   // input monitor through the tape sound 0..1, ramps like rec_gain
static float   vol_s = 1.0f;      // smoothed output volume
static float   mix_s = 0.5f;      // smoothed device / input mix
static float   mixmode_s = 0.0f;  // 0 = normal output, 1 = mix output; 10 ms ramp between them
static bool    head_ran = false;  // record head ran on the previous sample
static constexpr float kRecFade = 1.0f / 480.0f;   // overdub crossfade: 10 ms at 48 kHz
static constexpr float kOverdubKeep = 0.891f;     // overdub: old layers -1 dB per pass (switch up)

#ifdef CPU_TEST
static CpuLoadMeter cpu;   // audio callback load, CPU stress test only
// Cycles spent per part of the callback, per block (cycle counter), for the CPU test's breakdown.
enum { kProfGrains, kProfHead, kProfTape, kProfBlock, kProfCount };
static volatile bool prof_reset = true;   // set by the test loop, cleared by the callback
static uint64_t prof_sum[kProfCount];
static uint32_t prof_max[kProfCount], prof_blocks;
#define PROF_BEGIN(t) const uint32_t t = DWT->CYCCNT
#define PROF_END(part, t) prof_now[part] += DWT->CYCCNT - t
#else
#define PROF_BEGIN(t)
#define PROF_END(part, t)
#endif

#ifdef DEBUG_TRACE
// Diagnostic build, with (in template/): C_DEFS=-DDEBUG_TRACE make -s BUILD_DIR=build/debug
// TARGET=template_debug. Never part of the normal firmware. Breadcrumbs (debug_trace.h) in backup
// SRAM; a fault saves its registers and resets, a hang of more than 1 s resets through the
// watchdog. After the reset the device runs normally and prints the report over USB serial.
volatile DebugTrace __attribute__((section(".backup_sram"))) g_trace;
static constexpr uint32_t kTraceArmed = 0x7A9E5EEDu;
static DebugTrace trace_report;     // breadcrumbs from before the reset
static bool have_report = false, report_hang = false;
static constexpr int kVectors = 16 + 150;   // Cortex-M7 exceptions + STM32H750 interrupts
alignas(1024) static uint32_t ram_vectors[kVectors];

// Called with the stacked registers of the faulting code: r0-r3, r12, lr, pc, psr.
extern "C" __attribute__((used)) void TraceFault(uint32_t* frame) {
    g_trace.lr = frame[5];
    g_trace.pc = frame[6];
    g_trace.psr = frame[7];
    g_trace.cfsr = SCB->CFSR;
    g_trace.hfsr = SCB->HFSR;
    g_trace.bfar = SCB->BFAR;
    g_trace.mmfar = SCB->MMFAR;
    g_trace.cause = 1;
    __DSB();
    NVIC_SystemReset();
}

extern "C" __attribute__((naked)) void TraceFaultEntry() {
    __asm volatile(
        "tst lr, #4    \n"
        "ite eq        \n"
        "mrseq r0, msp \n"
        "mrsne r0, psp \n"
        "b TraceFault  \n");
}

// After hw.Init: keep a report from before the reset, re-arm, route the faults to TraceFault.
static void TraceBoot() {
    System::InitBackupSram();
    const uint32_t rsr = RCC->RSR;
    RCC->RSR |= RCC_RSR_RMVF;
    volatile uint32_t* src = reinterpret_cast<volatile uint32_t*>(&g_trace);
    uint32_t* dst = reinterpret_cast<uint32_t*>(&trace_report);
    const int words = sizeof(DebugTrace) / 4;
    if (g_trace.magic == kTraceArmed && (g_trace.cause == 1 || (rsr & RCC_RSR_IWDG1RSTF))) {
        for (int i = 0; i < words; i++) dst[i] = src[i];
        have_report = true;
        report_hang = (g_trace.cause != 1);
    }
    for (int i = 0; i < words; i++) src[i] = 0;
    g_trace.magic = kTraceArmed;
    const uint32_t* v = reinterpret_cast<const uint32_t*>(SCB->VTOR);
    for (int i = 0; i < kVectors; i++) ram_vectors[i] = v[i];
    for (int i = 3; i <= 6; i++)   // HardFault, MemManage, BusFault, UsageFault
        ram_vectors[i] = reinterpret_cast<uint32_t>(&TraceFaultEntry);
    SCB_CleanDCache_by_Addr(ram_vectors, sizeof(ram_vectors));
    __DSB();
    SCB->VTOR = reinterpret_cast<uint32_t>(ram_vectors);
    __DSB();
    __ISB();
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk;
}

// Independent watchdog: resets the device if the main loop stops refreshing it for 1 s.
static void TraceWatchdogStart() {
    IWDG1->KR = 0xCCCC;   // start
    IWDG1->KR = 0x5555;   // unlock the settings
    IWDG1->PR = 3;        // LSI 32 kHz / 32 = 1 kHz
    IWDG1->RLR = 1000;    // 1 s
    while (IWDG1->SR != 0) {}
    IWDG1->KR = 0xAAAA;   // refresh
}

static const char* TraceStageName(uint32_t s) {
    static const char* const names[] = {"none", "block start", "player", "grain start", "grain read",
                                        "head", "head ghost", "head write", "head stutter",
                                        "head lead zone", "tape sound", "block end"};
    return (s <= kTrBlockEnd) ? names[s] : "?";
}

#define F3(x) ((x) < 0.0f ? '-' : ' '), (int)fabsf(x), (int)((fabsf(x) - floorf(fabsf(x))) * 1000.0f)
static void TracePrint() {
    if (!have_report) {
        hw.PrintLine("diag build running, no report yet (blocks %lu)", (unsigned long)g_trace.blocks);
        return;
    }
    const DebugTrace& t = trace_report;
    hw.PrintLine("REPORT: %s in '%s', after %lu audio blocks, %lu main loops",
                 report_hang ? "HANG (watchdog)" : "FAULT", TraceStageName(t.stage),
                 (unsigned long)t.blocks, (unsigned long)t.main_loops);
    if (!report_hang)
        hw.PrintLine("  pc 0x%08lx lr 0x%08lx psr 0x%08lx cfsr 0x%08lx hfsr 0x%08lx bfar 0x%08lx mmfar 0x%08lx",
                     (unsigned long)t.pc, (unsigned long)t.lr, (unsigned long)t.psr, (unsigned long)t.cfsr,
                     (unsigned long)t.hfsr, (unsigned long)t.bfar, (unsigned long)t.mmfar);
    hw.PrintLine("  player: mode %ld loop %ld + %ld pos %c%d.%03d step %c%d.%03d speed %c%d.%03d",
                 (long)t.mode, (long)t.rs, (long)t.rl, F3(t.pos), F3(t.step), F3(t.speed));
    hw.PrintLine("  head: loop %ld + %ld pos %c%d.%03d step %c%d.%03d from %c%d.%03d count %ld k %ld write %ld",
                 (long)t.h_rs, (long)t.h_rl, F3(t.h_pos), F3(t.h_step), F3(t.from), (long)t.count,
                 (long)t.k, (long)t.wi);
    hw.PrintLine("  head jump: fade-in left %ld ghost left %ld ghost from %c%d.%03d",
                 (long)t.fade_left, (long)t.ghost_left, F3(t.ghost_from));
}
#endif

// S32: overdub aging per pass, scaled by tape type. At amount = 1 the old layers get a per-pass
// low-pass of 8 kHz (reel-to-reel) .. 1.5 kHz (dictaphone); amount = 0: no aging.
static void SetAgingFrom(float amount, float type) {
    const float f_min = 8000.0f * powf(1500.0f / 8000.0f, type);
    const float a_min = 1.0f - expf(-6.2831853f * f_min / 48000.0f);
    age_coef = 1.0f - amount * (1.0f - a_min);
    age_hiss = amount * (0.0003f + 0.0012f * type);   // per-pass hiss, before the low-pass
    // Printed stutter bursts per written sample: amount x 0.15/s (reel-to-reel) .. amount x
    // 1.5/s (dictaphone); each jerk peaks at up to amount x 0.5 % .. amount x 3 % pitch.
    age_wob = amount * 0.15f * powf(10.0f, type) / 48000.0f;
    age_wob_pitch = amount * (0.005f + 0.025f * type);
}

void AudioCallback(AudioHandle::InputBuffer in,
                   AudioHandle::OutputBuffer out,
                   size_t size) {
#ifdef CPU_TEST
    cpu.OnBlockStart();
    const uint32_t prof_block = DWT->CYCCNT;
    uint32_t prof_now[kProfCount] = {};
#endif
    TRACE_STAGE(kTrBlock);
    TRACE_SET(blocks, g_trace.blocks + 1);
    // 1. Copy flags into locals
    bool play = want_play;
    bool rec  = want_record;
    bool od   = want_overdub;

    // 2. Edge handling
    if (rec && !was_record) {
        rec_pos = 0;
        // The live input history becomes this recording's lead-in; write into the other ring.
        player.SetPreRoll(pre_l[pre_live], pre_r[pre_live], kPreRoll, pre_pos);
        head.SetPreRoll(pre_l[pre_live], pre_r[pre_live], kPreRoll, pre_pos);
        head.NewTape();
        pre_live = 1 - pre_live;
        pre_pos = 0;
        for (size_t j = 0; j < kOvdPre; j++) ovd_pre_l[j] = ovd_pre_r[j] = 0.0f;   // no overdub yet
    }
    if (!rec && was_record) {
        length = rec_pos;
        player.SetLength(length);
        player.Reset();
    }
    if (play && !was_play && !env.IsRunning()) {
        player.Reset();
    }
    was_play = play;
    was_record = rec;

    // 3. Pass parameters
    // Tape transport: battery -> tape speed, with wow/flutter for this tape type.
    const float type = tape_type;
    motor.SetBattery(battery);
    motor.SetWobble(TapeFormat::Wobble(type));
    head.SetAging(age_coef, age_hiss, age_wob, age_wob_pitch);
    const float vol = volume;
    const float kp = keep;
    const float mixv = mix;
    const float mixmode = mix_mode ? 1.0f : 0.0f;
    const float tape_speed = motor.Process();
    const float deficit    = motor.Deficit();
    TRACE_SET(speed, tape_speed);
    player.SetMode((GrainPlayer::Mode)mode);
    player.SetSpeed(tape_speed);
    // Varispeed: pitch follows tape speed (grains read at the tape speed, so they ride the playhead).
    player.SetRatio(tape_speed);
    player.SetRange(range_a, range_b);
    player.SetFadeMs(fade_ms);
    const float drv = drive, bump_amt = bump;
    fmt_l.SetParams(type, deficit, tape_speed, drv, bump_amt);
    fmt_r.SetParams(type, deficit, tape_speed, drv, bump_amt);
    const float tl = tilt;
    tilt_l.SetTilt(tl);
    tilt_r.SetTilt(tl);

    for (size_t i = 0; i < size; i++) {
        float in_l = in[0][i], in_r = in[1][i];
        pre_l[pre_live][pre_pos] = in_l;   // input history
        pre_r[pre_live][pre_pos] = in_r;
        if (++pre_pos >= kPreRoll) pre_pos = 0;
        if (rec) {
            if (rec_pos < kMaxSamples) {
                buf_l[rec_pos] = in_l;
                buf_r[rec_pos] = in_r;
                ovd_l[rec_pos] = 0.0f;   // a new recording has no overdub yet
                ovd_r[rec_pos] = 0.0f;
                rec_pos++;
            } else {
                want_record = false;   // buffer full: auto-stop, finalized next block
            }
        }
        float e = env.Process(play);
        // Overdub write level: 10 ms ramp on P11 on/off, times the play/stop declick.
        rec_gain = od ? fminf(rec_gain + kRecFade, 1.0f) : fmaxf(rec_gain - kRecFade, 0.0f);
        // Input monitor: through the tape sound while the input goes to tape, dry otherwise.
        const bool to_tape = rec || (od && play);
        mon_gain = to_tape ? fminf(mon_gain + kRecFade, 1.0f) : fmaxf(mon_gain - kRecFade, 0.0f);
        const float g = rec_gain * e;
        float l = 0.0f, r = 0.0f;
        bool head_now = false;
        if ((play || env.IsRunning()) && !rec) {
            PROF_BEGIN(t_grains);
            player.SetWriting(head_ran || g > 0.0f);   // loop changes wait while the head is at the loop edges
            player.Process(l, r);
            PROF_END(kProfGrains, t_grains);
            // Overdub: the record head follows the playhead, only while the tape moves.
            // Crossfade from the old layer to the new: old + g x (Sat(keep x old + input) - old).
            // Keep running after g reaches 0 until the head's delayed writes are done.
            if (g > 0.0f || (head_ran && head.Pending())) {
                PROF_BEGIN(t_head);
                if (!head_ran) head.Start();   // re-sync to the playhead after a pause
                head.Process(player.Pos(), player.Step(), player.Bounces(),
                             player.RegionStart(), player.RegionLen(), player.RegionXfade(), player.RegionOvdXfade(), player.RegionLead(),
                             in_l, in_r, g, kp);
                PROF_END(kProfHead, t_head);
                head_now = true;
            }
        }
        head_ran = head_now;
        // Tape format character on the grains (enveloped) and on the monitored input; its hiss
        // sounds while the tape plays or the input goes to tape.
        const float hiss = (e > mon_gain) ? e : mon_gain;
        TRACE_STAGE(kTrTape);
        PROF_BEGIN(t_tape);
        const float fl = tilt_l.Process(fmt_l.Process(l * e + mon_gain * in_l, hiss));
        const float fr = tilt_r.Process(fmt_r.Process(r * e + mon_gain * in_r, hiss));
        PROF_END(kProfTape, t_tape);
        vol_s += 0.002f * (vol - vol_s);   // smoothed output volume (P01 + S35)
        mix_s += 0.002f * (mixv - mix_s);
        mixmode_s = (mixmode > mixmode_s) ? fminf(mixmode_s + kRecFade, 1.0f) : fmaxf(mixmode_s - kRecFade, mixmode);
        // Normal: device + dry input monitor when not going to tape. Mix mode (right switch down):
        // equal-power mix of the device output and the dry input. 10 ms crossfade between the two.
        const float dev = FastSinHalfPi(1.0f - mix_s), dry = FastSinHalfPi(mix_s);
        const float nl = fl + (1.0f - mon_gain) * in_l, nr = fr + (1.0f - mon_gain) * in_r;
        const float ml = dev * fl + dry * in_l, mr = dev * fr + dry * in_r;
        out[0][i] = vol_s * (nl + mixmode_s * (ml - nl));
        out[1][i] = vol_s * (nr + mixmode_s * (mr - nr));
    }
    TRACE_STAGE(kTrBlockEnd);
#ifdef CPU_TEST
    prof_now[kProfBlock] = DWT->CYCCNT - prof_block;
    if (prof_reset) {
        for (int p = 0; p < kProfCount; p++) prof_sum[p] = prof_max[p] = 0;
        prof_blocks = 0;
        prof_reset = false;
    }
    prof_blocks++;
    for (int p = 0; p < kProfCount; p++) {
        prof_sum[p] += prof_now[p];
        if (prof_now[p] > prof_max[p]) prof_max[p] = prof_now[p];
    }
    cpu.OnBlockEnd();
#endif
}

#ifdef CPU_TEST
// CPU stress test, build with: C_DEFS=-DCPU_TEST make -C template. Never part of the normal
// firmware. Fills the tape with a loud test signal (a 10 s recording plus overdub layers), then
// cycles through worst-case scenarios with no controls touched and prints the audio callback
// load (average and peak) over USB serial.
struct CpuScenario {
    const char* name;
    int   mode;
    float battery, aging, type, fade, a, b;
    bool  play, overdub, moves;
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

static void CpuTestFill() {
    // Loud material (above the soft-limit knee) on both tracks and in the lead-ins.
    uint32_t seed = 1;
    const size_t len = 48000 * 10;
    for (size_t i = 0; i < len; i++) {
        seed = seed * 1664525u + 1013904223u;
        const float noise = (float)(int32_t)seed * (1.0f / 2147483648.0f);
        const float s = 0.6f * sinf(0.0287f * (float)i) + 0.3f * noise;
        buf_l[i] = buf_r[i] = s;
        ovd_l[i] = ovd_r[i] = 0.5f * sinf(0.0491f * (float)i) + 0.2f * noise;
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

static void CpuTestRun() {
    hw.StartLog(false);
    cpu.Init(hw.AudioSampleRate(), hw.AudioBlockSize());
    // Cycle counter for the per-part breakdown.
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55;   // unlock (Cortex-M7)
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    for (int cycle = 1;; cycle++) {
        System::Delay(2000);
        hw.PrintLine("CPU test, cycle %d (16-sample blocks at 48 kHz)", cycle);
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
            const uint32_t t0 = System::GetNow();
            bool reset = false;
            while (System::GetNow() - t0 < 5000) {
                if (!reset && System::GetNow() - t0 >= 1000) {   // 1 s to settle
                    cpu.Reset();
                    prof_reset = true;
                    reset = true;
                }
                if (s.moves) range_a = ((System::GetNow() / 400) % 2) ? 0.32f : 0.3f;
                System::Delay(5);
            }
            PrintLoad(s.name);
        }
    }
}
#endif

int main(void) {
    hw.Init(true);
#ifdef DEBUG_TRACE
    TraceBoot();
    hw.StartLog(false);
#endif
    hw.SetAudioBlockSize(16);
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

    // Clear SDRAM buffers
    for (size_t i = 0; i < kMaxSamples; i++) {
        buf_l[i] = 0.0f;
        buf_r[i] = 0.0f;
        ovd_l[i] = 0.0f;
        ovd_r[i] = 0.0f;
    }
    for (size_t i = 0; i < kPreRoll; i++) {
        pre_l[0][i] = pre_l[1][i] = 0.0f;
        pre_r[0][i] = pre_r[1][i] = 0.0f;
    }

    player.Init(buf_l, buf_r, kMaxSamples, hw.AudioSampleRate());
    player.SetOverdub(ovd_l, ovd_r, ovd_pre_l, ovd_pre_r, kOvdPre);
    player.SetGrainMs(100.0f);   // fixed: it only sets how long blends take at loop changes and ping-pong turns
    head.Init(buf_l, buf_r, ovd_l, ovd_r, kMaxSamples);
    head.SetOverdubPreRoll(ovd_pre_l, ovd_pre_r, kOvdPre);
    float sr = hw.AudioSampleRate();
    env.Init(sr);
    env.SetAttackTime(0.01f);    // play/stop declick: 10 ms in, 10 ms out
    env.SetReleaseTime(0.01f);
    env.SetDecayTime(0.01f);
    env.SetSustainLevel(1.0f);
    fmt_l.Init(sr, 1);
    fmt_r.Init(sr, 0x2545F491);   // different seed: independent hiss per channel
    tilt_l.Init(sr);
    tilt_r.Init(sr);
    motor.Init(hw.AudioCallbackRate());
    touch.Init(hw);
#ifdef CPU_TEST
    CpuTestFill();
    hw.StartAudio(AudioCallback);
    CpuTestRun();   // never returns
#endif
    hw.StartAudio(AudioCallback);

    // Settle baseline capacitance at boot
    for (int i = 0; i < 40; i++) {
        touch.Process();
        System::Delay(3);
    }

    // Pad callbacks (registered after settle)
    touch.pads().SetOnTouch([](uint16_t pad) {
        if (pad == 3) {                         // P03: play / stop (latched)
            want_play = !want_play;
        } else if (pad == 10) {                 // P10: record start / stop (latched)
            if (want_record) { want_record = false; }
            else { want_overdub = false; want_record = true; }
        } else if (pad == 11) {                 // P11: overdub start / stop (latched)
            if (want_overdub) { want_overdub = false; }
            else if (!want_record && length > 0) { want_overdub = true; }
        }
    });

    bool was_vol_mode = false, was_mixing = false;
#ifdef DEBUG_TRACE
    TraceWatchdogStart();
    uint32_t trace_t = System::GetNow();
#endif
    while (1) {
#ifdef DEBUG_TRACE
        IWDG1->KR = 0xAAAA;   // watchdog refresh
        g_trace.main_loops = g_trace.main_loops + 1;
        if (System::GetNow() - trace_t >= 1000) {
            trace_t = System::GetNow();
            TracePrint();
        }
#endif
        touch.Process();
        Knobs& k = touch.knobs();
        const float c31 = ctl[1].Process(k.s31());
        const float c32 = ctl[2].Process(k.s32());
        const float c33 = ctl[3].Process(k.s33());
        const float c34 = ctl[4].Process(k.s34());
        const float c35 = ctl[5].Process(k.s35());
        // Faders apply live: the player takes them at its next grain start.
        const float c36 = ctl[6].Process(k.LeftFader());
        const float c37 = ctl[7].Process(k.RightFader());
        // S30: tilt EQ on the device output, left darker .. middle flat .. right brighter.
        const float t30 = 2.0f * ctl[0].Process(k.s30()) - 1.0f;
        tilt     = (fabsf(t30) < 0.03f) ? 0.0f : t30;   // small dead zone: the middle is exactly flat
        battery  = c31;                               // S31: battery 0 dead, 0.5 fresh, 1 over-voltage
        SetAgingFrom(c32, c34);                       // S32: overdub aging per pass, scaled by tape type (S34)
        fade_ms  = 10.0f + 1990.0f * c33;             // S33: loop crossfade, 10 ms..2 s
        tape_type = c34;                              // S34: tape type 0 reel-to-reel .. 1 dictaphone
        // Right switch: overdub mode. Up = aging + -1 dB per pass, center = aging only, down = aging
        // only + P01 + S35 is the device / input mix.
        const int osw = touch.switches().B();
        keep = (osw == Switch3::POS_UP) ? kOverdubKeep : 1.0f;
        const bool mixing = (osw == Switch3::POS_DOWN);
        mix_mode = mixing;
        // S35 is shared: drive normally; while P01 is held, output volume (or the mix with the
        // right switch down). Pick-up on all jobs.
        const bool vol_mode = touch.pads().IsTouched(1);
        if (vol_mode != was_vol_mode || mixing != was_mixing) {
            drive_pick.Release();
            vol_pick.Release();
            mix_pick.Release();
            was_vol_mode = vol_mode;
            was_mixing = mixing;
        }
        const float c35d = vol_mode ? drive_pick.Value() : drive_pick.Process(c35);
        const float c35v = (vol_mode && !mixing) ? vol_pick.Process(c35) : vol_pick.Value();
        const float c35m = (vol_mode && mixing) ? mix_pick.Process(c35) : mix_pick.Value();
        volume   = 2.0f * c35v * c35v;                // P01 + S35: mute .. unity (0.71) .. +6 dB
        mix      = c35m;                              // P01 + S35, switch down: device (0) .. dry input (1)
        // S35: drive, times the drive from tape type and battery: 0.5x .. 1x (middle, as before) .. 4x.
        drive    = (c35d < 0.5f) ? 0.5f * powf(2.0f, 2.0f * c35d) : powf(4.0f, 2.0f * (c35d - 0.5f));
        // ...and right of the middle a head bump grows with it (0 .. +9 dB); middle = no bump, as before.
        bump     = (c35d > 0.5f) ? 2.0f * (c35d - 0.5f) : 0.0f;
        range_a  = c36;                               // S36: loop start, live
        range_b  = c37;                               // S37: loop length, 100 ms .. the rest after the start (exponential), live
        int sw = touch.switches().A();                    // left switch: play mode
        if (sw == Switch3::POS_UP)        mode = GrainPlayer::FORWARD;
        else if (sw == Switch3::POS_DOWN) mode = GrainPlayer::BACKWARD;
        else                              mode = GrainPlayer::PINGPONG;
        hw.SetLed(want_record || want_overdub);           // LED: recording or overdubbing
        System::Delay(3);
    }
}
