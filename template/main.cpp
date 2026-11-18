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
#include "live_grains.h"
#include "live_wear.h"
#include "window_shock.h"
#include "crush.h"
#include "beat_repeat.h"
#include "phaser.h"
#include "debug_trace.h"
#include "app.h"
#include "controls.h"
#include <cmath>

using namespace daisy;
using namespace daisysp;
using namespace synthux;

DaisySeed   hw;
GrainPlayer player;
static Adsr env;
static TapeFormat fmt_l, fmt_r;   // tape type character per channel
static TiltEq     tilt_l, tilt_r; // S30: tone tilt on the device output
static Motor      motor;          // one tape transport for both channels
RecordHead head;           // overdub record head, follows the playhead
TapeFrame DSY_SDRAM_BSS tape[kMaxSamples];   // the tape: recording + overdub track, both channels, per position
float ovd_pre_l[kOvdPre];
float ovd_pre_r[kOvdPre];
float DSY_SDRAM_BSS pre_l[2][kPreRoll];      // two rings: one live, one frozen
float DSY_SDRAM_BSS pre_r[2][kPreRoll];
static LiveGrains live;                             // reads the live input (the input history ring) at the tape speed
static int    live_ring = 0;     // input history ring the live grains read (kept while live mode fades out)
static size_t live_newest = 0;   // index of its newest sample
static constexpr size_t kRepeatRing = 48000 * 8 + 4096;   // beat repeat ring (fx box mode, and overdub): slices up to 8 s
static float DSY_SDRAM_BSS rep_l[kRepeatRing];
static float DSY_SDRAM_BSS rep_r[kRepeatRing];
static BeatRepeat repeat;                           // P07 held: beat repeat
static LiveWear   wear;                             // S32 on the live input: noise, stutter
static WindowShock shock;                           // P08 held: damaged tape under the head
static Crush       crush;                           // P02 held: bits / sample rate down on the output
static ::Phaser    phaser;                          // P05 held: vintage 4-stage phaser on the output

// Written by main loop, read by audio callback
volatile bool  want_play    = false;
volatile bool  want_record  = false;
volatile bool  want_overdub = false;
volatile bool  want_live    = false;   // live input through the tape (P11 with nothing recorded)
volatile float battery = 0.5f;
volatile float friction = 0.0f;   // P09 pressure: finger on the tape, extra motor load 0 .. 2
volatile float shock_sev = 0.0f;  // P08 pressure: window shock severity 0 .. 1
volatile float crush_amt = 0.0f;  // P02 pressure: crush amount 0 .. 1
volatile float phaser_p = 0.0f;   // P05 pressure: phaser depth + speed 0 .. 1
volatile bool  rep_on = false;    // P07 touched: beat repeat
volatile float rep_div = 4.0f;    // its division of the loop: 4, 8, 16, 32 (P07 pressure)
volatile float glitch_amt = 0.0f; // P06: share of grains that jump (glitch) 0 .. 1
volatile float range_a = 0.0f, range_b = 1.0f;
volatile int   mode = GrainPlayer::FORWARD;
volatile float fade_ms = 200.0f, tape_type = 0.33f;
volatile float drive = 1.0f;    // S35: multiplier on the tape saturation drive, 0.5 .. 4
volatile float bump = 0.0f;     // S35 right of middle: head bump amount 0 .. 1
volatile float tilt = 0.0f;     // S30: tilt EQ, -1 darker .. 0 flat .. 1 brighter
volatile float age_coef = 1.0f, age_hiss = 0.0f, age_wob = 0.0f, age_wob_pitch = 0.0f;   // overdub aging per pass (S32)
volatile float age_amount = 0.0f;   // S32 as set (live input wear)
volatile float volume = 1.0f;   // output volume, 0 .. 2 (P01 + S35)
// Right switch = overdub mode: up = aging + volume drop per pass, center = aging only, down = aging
// only and P01 + S35 mixes the device output (0) with the dry input (1).
volatile float keep = 0.891f;   // share of the old layers kept per pass (1 = no drop)
volatile bool  mix_mode = false;
volatile float mix = 0.5f;      // device / dry input mix in mix_mode, 0 .. 1

// Written by audio callback, read by main loop
volatile size_t length = 0;   // recorded samples per channel

// Callback-only state
static bool    was_play = false, was_record = false;
static size_t  rec_pos = 0;
int     pre_live = 0;      // ring the input is written to
static size_t  pre_pos = 0;       // next write index in the live ring (= its oldest sample)
static float   rec_gain = 0.0f;   // overdub write level 0..1, ramps on P11 on/off
static float   mon_gain = 0.0f;   // input monitor through the tape sound 0..1, ramps like rec_gain
static float   vol_s = 1.0f;      // smoothed output volume
static float   mix_s = 0.5f;      // smoothed device / input mix
static float   mixmode_s = 0.0f;  // 0 = normal output, 1 = mix output; 10 ms ramp between them
static bool    head_ran = false;  // record head ran on the previous sample
static bool    was_live = false;
static float   live_gain = 0.0f;  // live input through the tape 0..1, 10 ms ramp
static float   prt_l[32] = {}, prt_r[32] = {};   // printing effects: heard minus tape, delayed to the record head
static int     prt_i = 0;                        // next write index in that ring
static float   print_w = 0.0f;                   // printing weight 0..1 (10 ms in, 100 ms out)
static float   gin_w = 0.0f;                     // glitch on the overdub input 0..1 (10 ms ramp)
static bool    was_gin = false;
static constexpr float kRecFade = 1.0f / 480.0f;   // overdub crossfade: 10 ms at 48 kHz
#include "cpu_test.h"   // CPU test build: load meter, per-part profiling (PROF_BEGIN / PROF_END)

// The input: into the input history ring (a recording's lead-in, fx box mode's source) and,
// while recording (P10), onto the tape.
static void RecordInput(float in_l, float in_r, bool rec) {
    pre_l[pre_live][pre_pos] = in_l;   // input history
    pre_r[pre_live][pre_pos] = in_r;
    if (++pre_pos >= kPreRoll) pre_pos = 0;
    if (rec) {
        if (rec_pos < kMaxSamples) {
            tape[rec_pos] = {in_l, in_r, 0.0f, 0.0f};   // a new recording has no overdub yet
            rec_pos++;
        } else {
            want_record = false;   // buffer full: auto-stop, finalized next block
        }
    }
}

// The output: the device sound (fl, fr) with the dry input, volume (P01 + S35) and the device /
// input mix (right switch down).
static void OutputMix(float fl, float fr, float in_l, float in_r, float vol, float mixv, float mixmode,
                      float& out_l, float& out_r) {
    vol_s += 0.002f * (vol - vol_s);   // smoothed output volume (P01 + S35)
    mix_s += 0.002f * (mixv - mix_s);
    mixmode_s = (mixmode > mixmode_s) ? fminf(mixmode_s + kRecFade, 1.0f) : fmaxf(mixmode_s - kRecFade, mixmode);
    // Normal: device + dry input monitor when not going to tape. Mix mode (right switch down):
    // equal-power mix of the device output and the dry input. 10 ms crossfade between the two.
    const float dev = FastSinHalfPi(1.0f - mix_s), dry = FastSinHalfPi(mix_s);
    const float dry_g = fmaxf(1.0f - mon_gain - live_gain, 0.0f);   // dry input while not through the tape
    const float nl = fl + dry_g * in_l, nr = fr + dry_g * in_r;
    const float ml = dev * fl + dry * in_l, mr = dev * fr + dry * in_r;
    out_l = vol_s * (nl + mixmode_s * (ml - nl));
    out_r = vol_s * (nr + mixmode_s * (mr - nr));
}

// One audio block's settings: the transport flags, plus the values the per-sample loop needs.
struct Block {
    bool  play, rec, od, live_on;            // transport flags, copied once per block
    bool  print_on;                          // overdub with a printable pad effect held
    bool  live_run, gin_on, live_proc;       // live grains: fx box mode / glitch on the overdub input
    float head_lvl;                          // head output level (follows the tape speed below 0.2)
    float kp, vol, mixv, mixmode;            // overdub keep, output volume, device / input mix
    int   prt_d0;                            // printed effects: delay to the record head,
    float prt_fr;                            // whole samples + fraction
};

// Transport edges (record start / stop, play start), then this block's parameters for all the
// DSP objects (motor, player, record head, effects, tape sound, live grains).
static Block SetupBlock(size_t size) {
    // 1. Copy flags into locals
    bool play = want_play;
    bool rec  = want_record;
    bool od   = want_overdub;
    bool live_on = want_live;

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
    motor.SetFriction(friction);
    motor.SetWobble(TapeFormat::Wobble(type));
    head.SetAging(age_coef, age_hiss, age_wob, age_wob_pitch);
    const float vol = volume;
    const float kp = keep;
    const float mixv = mix;
    const float mixmode = mix_mode ? 1.0f : 0.0f;
    const float tape_speed = motor.Process() * shock.SpeedFactor((int)size);   // + capstan hiccups (P08)
    const float deficit    = motor.Deficit();
    // Head output follows the tape speed below 0.2: a stopped tape is silent (a dead battery,
    // ~0.23, is above it).
    const float head_lvl = fminf(5.0f * tape_speed, 1.0f);
    // Window shock (P08) only while the tape moves, weaker with the head output.
    shock.SetSeverity((play || live_on) ? shock_sev * head_lvl : 0.0f);
    crush.SetAmount(crush_amt);
    phaser.SetPressure(phaser_p, (int)size);
    // Printing effects (overdub): while overdubbing with a printable pad effect held (not the
    // finger: slowing the tape doesn't change what's on it), what is heard is written back.
    const bool print_on = od && (glitch_amt > 0.0f || rep_on || shock_sev > 0.0f || phaser_p > 0.0f || crush_amt > 0.0f);
    player.SetPrint(print_on || print_w > 0.0f);
    // The record head writes 8 samples behind the playhead (RecordHead's kLag), sample k at k + 0.5.
    const float prt_d = fminf(8.0f + 0.5f / fmaxf(tape_speed, 0.1f), 30.0f);
    const int prt_d0 = (int)prt_d;
    const float prt_fr = prt_d - (float)prt_d0;
    // Beat repeat (P07): in fx box mode and while overdubbing, the BeatRepeat ring repeats the
    // signal (overdub: tape + input, so both repeat and get recorded); otherwise the grain player
    // repeats a slice of the tape itself. Slice = loop / division (tape samples; in the ring as
    // time at this tape speed); fx box mode a nominal 1 s bar / division. Only while it moves.
    const bool rep_ring = live_on || od;
    player.SetRepeat(rep_on && play && !rep_ring, (double)player.RegionLen() / rep_div);
    const float ring_slice = live_on ? 48000.0f / rep_div
                                     : (float)player.RegionLen() / fmaxf(tape_speed, 0.1f) / rep_div;
    repeat.Set(rep_on && rep_ring && (play || live_on), ring_slice);
    player.SetGlitch(glitch_amt);   // P06: grains jump to random places (tape and fx box mode)
    live.SetGlitch(glitch_amt);
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
    // Live input through the tape: grains read the input at the tape speed (pitch follows the
    // battery, wow and flutter; tempo stays live). The ring keeps running while there is no tape.
    wear.SetAging(age_amount, age_coef, age_hiss, age_wob, age_wob_pitch);
    // Fx box mode: tape speed + stutter jerks (S32). Glitch on the overdub input: speed 1.
    const float wear_f = wear.SpeedFactor((int)size);
    live.SetRatio((live_on || live_gain > 0.0f) ? tape_speed * wear_f : 1.0f);
    // The live grains read the input history ring; while live mode is on or fading out they stay
    // on their ring (a record start switches the writing to the other ring).
    if (!(live_on || live_gain > 0.0f)) live_ring = pre_live;
    live.SetRing(pre_l[live_ring], pre_r[live_ring], kPreRoll);
    if (live_on && !was_live) live.Reset();   // grains start at the delay behind the newest input
    was_live = live_on;
    const bool live_run = (length == 0) || live_on || live_gain > 0.0f;
    // Glitch (P06) on the input while overdubbing: the live grains read the input's recent history
    // with the glitch jumps; crossfaded in for as long as glitch is held.
    const bool gin_on = od && glitch_amt > 0.0f;
    if (gin_on && !was_gin && !live_run) live.Reset();
    was_gin = gin_on;
    const bool live_proc = live_run || gin_on || gin_w > 0.0f;
    return {play, rec, od, live_on, print_on, live_run, gin_on, live_proc,
            head_lvl, kp, vol, mixv, mixmode, prt_d0, prt_fr};
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
    const auto [play, rec, od, live_on, print_on, live_run, gin_on, live_proc,
                head_lvl, kp, vol, mixv, mixmode, prt_d0, prt_fr] = SetupBlock(size);

    for (size_t i = 0; i < size; i++) {
        float in_l = in[0][i], in_r = in[1][i];
        RecordInput(in_l, in_r, rec);
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
                // + the printed effects (what was heard minus the tape), delayed to this head position.
                const float pa_l = prt_l[(prt_i - prt_d0) & 31], pb_l = prt_l[(prt_i - prt_d0 - 1) & 31];
                const float pa_r = prt_r[(prt_i - prt_d0) & 31], pb_r = prt_r[(prt_i - prt_d0 - 1) & 31];
                head.Process(player.Pos(), player.Step(), player.Bounces(),
                             player.RegionStart(), player.RegionLen(), player.RegionXfade(), player.RegionOvdXfade(), player.RegionLead(),
                             in_l + pa_l + prt_fr * (pb_l - pa_l), in_r + pa_r + prt_fr * (pb_r - pa_r), g, kp);
                PROF_END(kProfHead, t_head);
                head_now = true;
            }
        }
        head_ran = head_now;
        // Tape format character on the grains (enveloped) and on the monitored input; its hiss
        // sounds while the tape plays or the input goes to tape.
        live_gain = live_on ? fminf(live_gain + kRecFade, 1.0f) : fmaxf(live_gain - kRecFade, 0.0f);
        float lv_l = 0.0f, lv_r = 0.0f;
        if (live_proc) {
            PROF_BEGIN(t_live);
            if (live_ring == pre_live) live_newest = (pre_pos > 0) ? pre_pos - 1 : kPreRoll - 1;
            live.Process(live_newest, lv_l, lv_r);
            if (live_run) wear.Process(lv_l, lv_r);   // generation noise (S32), fx box mode only
            PROF_END(kProfGrains, t_live);
        }
        const float hiss = fmaxf(fmaxf(e * head_lvl, mon_gain), live_gain * head_lvl);
        TRACE_STAGE(kTrTape);
        PROF_BEGIN(t_tape);
        // Tape signal (playback + live input), with the window shock (P08) on it.
        // While overdubbing the input joins the tape signal before the pad effects (beat repeat,
        // window shock): they act on both and both get recorded. Otherwise it is added after them.
        const float mi = od ? mon_gain : 0.0f;
        // Glitch on the overdub input: crossfade to the live grains' (glitched) input.
        gin_w = gin_on ? fminf(gin_w + kRecFade, 1.0f) : fmaxf(gin_w - kRecFade, 0.0f);
        const float gi_l = in_l + gin_w * (lv_l - in_l), gi_r = in_r + gin_w * (lv_r - in_r);
        float tp_l = head_lvl * (l * e + live_gain * lv_l) + mi * gi_l;
        float tp_r = head_lvl * (r * e + live_gain * lv_r) + mi * gi_r;
        if (live_run || od) repeat.Process(tp_l, tp_r);   // P07 in fx box mode and overdub (else: in the grain player)
        shock.Process(tp_l, tp_r);
        float fl = tilt_l.Process(fmt_l.Process(tp_l + (mon_gain - mi) * in_l, hiss));
        float fr = tilt_r.Process(fmt_r.Process(tp_r + (mon_gain - mi) * in_r, hiss));
        const float fl0 = fl, fr0 = fr;   // before the post effects (for printing)
        phaser.Process(fl, fr);  // P05: phaser on the device output (after the tape sound and tilt)
        crush.Process(fl, fr);   // P02: crush on the device output (not the dry input)
        // Printing: what is heard minus the plain tape under the playhead (pre-tape effects on the
        // tape signal + post effects on the output), into the ring the record head reads.
        print_w = print_on ? fminf(print_w + kRecFade, 1.0f) : fmaxf(print_w - 0.1f * kRecFade, 0.0f);
        float pd_l = 0.0f, pd_r = 0.0f;
        if (print_w > 0.0f) {
            pd_l = print_w * ((tp_l - head_lvl * e * player.PlainL() - mi * in_l) + (fl - fl0));
            pd_r = print_w * ((tp_r - head_lvl * e * player.PlainR() - mi * in_r) + (fr - fr0));
        }
        prt_l[prt_i] = pd_l;
        prt_r[prt_i] = pd_r;
        prt_i = (prt_i + 1) & 31;
        PROF_END(kProfTape, t_tape);
        OutputMix(fl, fr, in_l, in_r, vol, mixv, mixmode, out[0][i], out[1][i]);
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

int main(void) {
    hw.Init(true);
#ifdef DEBUG_TRACE
    TraceBoot();
    hw.StartLog(false);
#endif
    hw.SetAudioBlockSize(32);   // per-block work (coefficients, settings) 1500 times a second
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

    // Clear SDRAM buffers
    for (size_t i = 0; i < kMaxSamples; i++) {
        tape[i] = {0.0f, 0.0f, 0.0f, 0.0f};
    }
    for (size_t i = 0; i < kPreRoll; i++) {
        pre_l[0][i] = pre_l[1][i] = 0.0f;
        pre_r[0][i] = pre_r[1][i] = 0.0f;
    }

    player.Init(tape, kMaxSamples, hw.AudioSampleRate());
    player.SetOverdub(ovd_pre_l, ovd_pre_r, kOvdPre);
    player.SetGrainMs(100.0f);   // fixed: it only sets how long blends take at loop changes and ping-pong turns
    head.Init(tape, kMaxSamples);
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
    live.Init(sr);
    repeat.Init(rep_l, rep_r, kRepeatRing);
    motor.Init(hw.AudioCallbackRate());
    SetupControls();   // touch hardware + pad calibration (controls.cpp)
    shock.Init(sr);
    phaser.Init(sr);
#ifdef CPU_TEST
    CpuTestFill();
    hw.StartAudio(AudioCallback);
#ifdef FUZZ_TEST
    CpuFuzzRun();   // never returns
#else
    CpuTestRun();   // never returns
#endif
#endif
    hw.StartAudio(AudioCallback);

    StartControls();   // settle the pads, pad press handler (controls.cpp)
#ifdef DEBUG_TRACE
    TraceWatchdogStart(1000);
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
        ReadControls();   // knobs, faders, switches, pads -> the audio callback's values, LED (controls.cpp)
        TRACE_SET(main_stage, 3);
        System::Delay(3);
        TRACE_SET(main_stage, 4);
    }
}
