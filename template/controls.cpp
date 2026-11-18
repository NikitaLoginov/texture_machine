// Controls: the Touch 2's knobs, faders, switches and pads -> the values in app.h that the audio
// callback reads, the pad press handler (P03 play, P10 record, P11 overdub / fx box mode), the LED.
#include "app.h"
#include "controls.h"
#include "control_filter.h"
#include "touch/touch.h"
#include "debug_trace.h"
#include <cmath>

using namespace daisy;
using namespace synthux;

static Touch       touch;
static ControlFilter ctl[8];   // S30..S35, left fader (S36), right fader (S37)
static Pickup drive_pick(0.5f, true);     // S35 normal job: drive
static Pickup vol_pick(0.7071f, false);   // S35 while P01 is held: output volume (starts at unity)
static Pickup mix_pick(0.5f, false);      // S35 while P01 is held, right switch down: device / input mix
static constexpr float kOverdubKeep = 0.891f;     // overdub: old layers -1 dB per pass (switch up)
static bool was_vol_mode = false, was_mixing = false;   // S35's job last pass (pick-up)

// S32: overdub aging per pass, scaled by tape type. At amount = 1 the old layers get a per-pass
// low-pass of 8 kHz (reel-to-reel) .. 1.5 kHz (dictaphone); amount = 0: no aging.
void SetAgingFrom(float amount, float type) {
    const float f_min = 8000.0f * powf(1500.0f / 8000.0f, type);
    const float a_min = 1.0f - expf(-6.2831853f * f_min / 48000.0f);
    age_coef = 1.0f - amount * (1.0f - a_min);
    age_amount = amount;
    age_hiss = amount * (0.0003f + 0.0012f * type);   // per-pass hiss, before the low-pass
    // Printed stutter bursts per written sample: amount x 0.15/s (reel-to-reel) .. amount x
    // 1.5/s (dictaphone); each jerk peaks at up to amount x 0.5 % .. amount x 3 % pitch.
    age_wob = amount * 0.15f * powf(10.0f, type) / 48000.0f;
    age_wob_pitch = amount * (0.005f + 0.025f * type);
}

// Before hw.StartAudio.
void SetupControls() {
    touch.Init(hw);
    touch.pads().SetMaxDelta(9, 530.0f);   // P09 (finger on the tape): measured light 5-120, medium ~410, hard 505-527
    touch.pads().SetMaxDelta(8, 540.0f);   // P08 (window shock): measured light 20-145, medium ~500, hard 530-538
    touch.pads().SetMaxDelta(2, 535.0f);   // P02 (crush): like P08/P09 (their hard press tops out at ~525-535)
    touch.pads().SetMaxDelta(5, 535.0f);   // P05 (phaser): like P08/P09
    touch.pads().SetMaxDelta(7, 535.0f);   // P07 (beat repeat): like P08/P09
    touch.pads().SetMaxDelta(6, 535.0f);   // P06 (glitch): like P08/P09
}

// After hw.StartAudio.
void StartControls() {
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
            else { want_overdub = false; want_live = false; want_record = true; }
        } else if (pad == 11) {                 // P11: overdub start / stop (latched); nothing recorded: live input through the tape
            if (want_overdub) { want_overdub = false; }
            else if (want_live) { want_live = false; }
            else if (!want_record && length > 0) { want_overdub = true; }
            else if (!want_record) { want_live = true; }
        }
    });
}

// Every main loop pass.
void ReadControls() {
    TRACE_SET(main_stage, 1);
    touch.Process();
    TRACE_SET(main_stage, 2);
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
    const float p09 = touch.pads().Pressure(9);
    friction = 2.0f * p09 * p09 * p09;            // P09: finger on the tape (pressure 0..1, gentle start); full press: 9-11 % speed
    shock_sev = touch.pads().Pressure(8);         // P08: window shock (damaged tape), pressure = severity
    crush_amt = touch.pads().Pressure(2);         // P02: crush, pressure = amount
    phaser_p = touch.pads().Pressure(5);          // P05: phaser, pressure = depth + speed
    // P07: beat repeat while touched; pressure picks the division (harder = shorter slice).
    rep_on = touch.pads().IsTouched(7);
    const float n07 = sqrtf(touch.pads().Pressure(7));
    rep_div = (n07 < 0.4f) ? 4.0f : (n07 < 0.6f) ? 8.0f : (n07 < 0.8f) ? 16.0f : 32.0f;
    glitch_amt = sqrtf(touch.pads().Pressure(6));   // P06: glitch, 0 when not touched
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
    hw.SetLed(want_record || want_overdub || want_live);   // LED: recording, overdubbing or live input
}
