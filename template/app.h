#pragma once
// Shared state of the instrument, defined in main.cpp: the hardware, the tape (buffers, player,
// record head) and the values the main loop and the test builds set for the audio callback.
#include "daisy_seed.h"
#include "grain_player.h"
#include "record_head.h"
#include "tape.h"
#include <cstddef>

extern daisy::DaisySeed hw;
extern GrainPlayer      player;   // plays the tape (grains)
extern RecordHead       head;     // overdub record head

// The tape: recording + overdub track, their lead-ins, the input history.
constexpr size_t kMaxSamples = 48000 * 30;   // 30 s per channel
constexpr size_t kOvdPre = 2048;             // overdub track lead-in before sample 0
constexpr size_t kPreRoll = 48000 * 2;       // 2 s input history = lead-in of a recording
extern TapeFrame tape[kMaxSamples];   // recording + overdub track, both channels, per position
extern float ovd_pre_l[kOvdPre], ovd_pre_r[kOvdPre];
extern float pre_l[2][kPreRoll], pre_r[2][kPreRoll];   // two rings: one live, one frozen
extern int   pre_live;                                 // ring the input is written to

// Written by the main loop (or a test build), read by the audio callback.
extern volatile bool  want_play, want_record, want_overdub, want_live;
extern volatile float battery, friction;
extern volatile float shock_sev, crush_amt, phaser_p, glitch_amt;   // pad effects (pressure)
extern volatile bool  rep_on;
extern volatile float rep_div;
extern volatile float range_a, range_b;
extern volatile int   mode;
extern volatile float fade_ms, tape_type, drive, bump, tilt;
extern volatile float age_coef, age_hiss, age_wob, age_wob_pitch, age_amount;
extern volatile float volume, keep, mix;
extern volatile bool  mix_mode;

// Written by the audio callback, read by the main loop.
extern volatile size_t length;   // recorded samples per channel

// S32 degradation and tape type -> the aging values above.
void SetAgingFrom(float amount, float type);
