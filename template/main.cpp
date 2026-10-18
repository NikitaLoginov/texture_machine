#include "daisy_seed.h"
#include "daisysp.h"
#include "touch/touch.h"
#include "grain_player.h"
#include "control_filter.h"
#include "motor.h"
#include "tape_format.h"
#include <cmath>

using namespace daisy;
using namespace daisysp;
using namespace synthux;

static DaisySeed   hw;
static Touch       touch;
static GrainPlayer player;
static Adsr env;
static TapeFormat fmt_l, fmt_r;   // tape type character per channel
static Motor      motor;          // one tape transport for both channels
static ControlFilter ctl[8];   // S30..S35, left fader (S36), right fader (S37)

static constexpr size_t kMaxSamples = 48000 * 30;   // 30 s per channel
static float DSY_SDRAM_BSS buf_l[kMaxSamples];
static float DSY_SDRAM_BSS buf_r[kMaxSamples];

// Written by main loop, read by audio callback
static volatile bool  want_play    = false;
static volatile bool  want_record  = false;
static volatile bool  want_overdub = false;
static volatile float grain_ms = 100.0f, battery = 0.5f, pitch = 0.0f;
static volatile float range_a = 0.0f, range_b = 1.0f;
static volatile int   mode = GrainPlayer::FORWARD;
static volatile float fade_ms = 200.0f, tape_type = 0.33f, res = 0.0f;

// Written by audio callback, read by main loop
static volatile size_t length = 0;   // recorded samples per channel

// Callback-only state
static bool    was_play = false, was_record = false, was_overdub = false;
static size_t  rec_pos = 0, od_pos = 0;

void AudioCallback(AudioHandle::InputBuffer in,
                   AudioHandle::OutputBuffer out,
                   size_t size) {
    // 1. Copy flags into locals
    bool play = want_play;
    bool rec  = want_record;
    bool od   = want_overdub;

    // 2. Edge handling
    if (rec && !was_record) {
        rec_pos = 0;
    }
    if (!rec && was_record) {
        length = rec_pos;
        player.SetLength(length);
        player.Reset();
    }
    if (od && !was_overdub) {
        od_pos = 0;
    }
    if (play && !was_play && !env.IsRunning()) {
        player.Reset();
    }
    was_play = play;
    was_record = rec;
    was_overdub = od;

    // 3. Pass parameters
    // Tape transport: battery -> tape speed, with wow/flutter for this tape type.
    const float type = tape_type;
    motor.SetBattery(battery);
    motor.SetWobble(TapeFormat::Wobble(type));
    const float tape_speed = motor.Process();
    const float deficit    = motor.Deficit();
    player.SetMode((GrainPlayer::Mode)mode);
    player.SetSpeed(tape_speed);
    // Varispeed: pitch follows tape speed; S32 adds an offset.
    player.SetPitch(pitch + 12.0f * log2f(tape_speed > 0.01f ? tape_speed : 0.01f));
    player.SetGrainMs(grain_ms);
    player.SetRange(range_a, range_b);
    player.SetFadeMs(fade_ms);
    fmt_l.SetParams(type, deficit, tape_speed, res);
    fmt_r.SetParams(type, deficit, tape_speed, res);

    for (size_t i = 0; i < size; i++) {
        float in_l = in[0][i], in_r = in[1][i];
        if (rec) {
            if (rec_pos < kMaxSamples) {
                buf_l[rec_pos] = in_l;
                buf_r[rec_pos] = in_r;
                rec_pos++;
            } else {
                want_record = false;   // buffer full: auto-stop, finalized next block
            }
        } else if (od && length > 0) {
            if (od_pos >= length) od_pos = 0;
            buf_l[od_pos] += in_l;
            buf_r[od_pos] += in_r;
            od_pos++;
        }
        float e = env.Process(play);
        float l = 0.0f, r = 0.0f;
        if ((play || env.IsRunning()) && !rec) player.Process(l, r);
        const float fl = fmt_l.Process(l), fr = fmt_r.Process(r);   // tape format character
        out[0][i] = fl * e + in_l;   // grains (tape format, enveloped) + dry input monitor
        out[1][i] = fr * e + in_r;
    }
}

int main(void) {
    hw.Init(true);
    hw.SetAudioBlockSize(16);
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

    // Clear SDRAM buffers
    for (size_t i = 0; i < kMaxSamples; i++) {
        buf_l[i] = 0.0f;
        buf_r[i] = 0.0f;
    }

    player.Init(buf_l, buf_r, kMaxSamples, hw.AudioSampleRate());
    float sr = hw.AudioSampleRate();
    env.Init(sr);
    env.SetAttackTime(0.01f);    // play/stop declick: 10 ms in, 10 ms out
    env.SetReleaseTime(0.01f);
    env.SetDecayTime(0.01f);
    env.SetSustainLevel(1.0f);
    fmt_l.Init(sr, 1);
    fmt_r.Init(sr, 0x2545F491);   // different seed: independent hiss per channel
    motor.Init(hw.AudioCallbackRate());
    touch.Init(hw);
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

    while (1) {
        touch.Process();
        Knobs& k = touch.knobs();
        const float c30 = ctl[0].Process(k.s30());
        const float c31 = ctl[1].Process(k.s31());
        const float c32 = ctl[2].Process(k.s32());
        const float c33 = ctl[3].Process(k.s33());
        const float c34 = ctl[4].Process(k.s34());
        const float c35 = ctl[5].Process(k.s35());
        const float c36 = ctl[6].Process(k.LeftFader());
        const float c37 = ctl[7].Process(k.RightFader());
        grain_ms = 10.0f * powf(50.0f, c30);          // S30: 10..500 ms, exponential
        battery  = c31;                               // S31: battery 0 dead, 0.5 fresh, 1 over-voltage
        float st = (c32 - 0.5f) * 24.0f;              // S32: -12..+12 semitones
        if (fabsf(st) < 0.25f) st = 0.0f;             // snap to 0 near center
        pitch    = st;
        fade_ms  = 10.0f + 1990.0f * c33;             // S33: loop-edge fade, 10 ms..2 s
        tape_type = c34;                              // S34: tape type 0 reel-to-reel .. 1 dictaphone
        float q  = 0.5f + 10.0f * c35;                // S35: filter Q 0.5..10.5
        float rr = 1.0f - 0.5f / q;                   // Svf resonance giving that Q
        res      = rr * rr * rr * rr;
        range_a  = c36;                               // S36: loop start
        range_b  = c37;                               // S37: loop end
        int sw = touch.switches().A();                    // left switch: play mode
        if (sw == Switch3::POS_UP)        mode = GrainPlayer::FORWARD;
        else if (sw == Switch3::POS_DOWN) mode = GrainPlayer::BACKWARD;
        else                              mode = GrainPlayer::PINGPONG;
        hw.SetLed(want_record || want_overdub);           // LED: recording or overdubbing
        System::Delay(3);
    }
}
