#include "tape_format.h"
#include "fast_math.h"
#include <cmath>
using namespace daisysp;

// Steady hiss level: about -69 dBFS (hi-fi) .. -71 dBFS (lo-fi) RMS after its low-pass.
static constexpr float kHissLevel = 0.001f;
// Make-up gain never lifts quiet parts (and noise) by more than this: +8 dB.
static constexpr float kMaxLift = 2.512f;

void TapeFormat::Init(float sample_rate, int32_t noise_seed) {
    sr_ = sample_rate;
    hp_.Init(sample_rate);
    lp_.Init(sample_rate);
    hiss_lp_.Init(sample_rate);
    noise_.Init();
    noise_.SetSeed(noise_seed);
}

float TapeFormat::Wobble(float type) {
    return 0.0003f * powf(16.67f, type);  // 0.03 % (reel) .. 0.5 % (dictaphone)
}

void TapeFormat::SetParams(float type, float deficit, float speed, float drive, float bump) {
    // Band edges per format, moving with tape speed: reel 25 Hz-16 kHz .. dictaphone 300 Hz-4 kHz.
    const float hp_freq = 25.0f * powf(12.0f, type) * speed;
    hp_.SetFreq(hp_freq);
    hp_.SetRes(0.0f);
    lp_.SetFreq(16000.0f * powf(0.25f, type) * speed);
    lp_.SetRes(0.0f);
    // Saturation drive 1 .. 4 from tape type and a weak battery, times the S35 drive (0.5 .. 4).
    drive_ = (1.0f + 1.5f * type + 1.5f * deficit) * drive;
    // Make-up gain: a 0.5 peak comes out at 0.5, but quiet parts are lifted by at most kMaxLift
    // (up to drive 4 this is today's rule; harder drive then gets a little quieter instead of
    // lifting the noise further).
    norm_ = fminf(0.5f / tanhf(0.5f * drive_), kMaxLift / drive_);
    // Head bump: resonant peak just above the tape's low cut (twice the high-pass frequency, so it
    // moves with tape type and speed), Q 2, 0 .. +9 dB (RBJ peaking EQ).
    const float f0 = fminf(2.0f * hp_freq, 0.45f * sr_);
    const float a = powf(10.0f, 9.0f * bump / 40.0f);
    const float w0 = 6.2831853f * f0 / sr_;
    const float alpha = sinf(w0) / (2.0f * 2.0f);
    const float cw = cosf(w0);
    const float a0 = 1.0f + alpha / a;
    bb0_ = (1.0f + alpha * a) / a0;
    bb1_ = -2.0f * cw / a0;
    bb2_ = (1.0f - alpha * a) / a0;
    ba1_ = bb1_;
    ba2_ = (1.0f - alpha / a) / a0;
    // Hiss low-pass: 12 kHz (hi-fi) .. 5 kHz (lo-fi). Not affected by speed or battery.
    hiss_lp_.SetFreq(12000.0f * powf(5.0f / 12.0f, type));
    hiss_lp_.SetRes(0.0074f);   // Q 0.707: flat up to the cutoff
}

float TapeFormat::Process(float in, float hiss) {
    const float x = FastTanh(drive_ * in) * norm_;   // saturation with make-up gain
    hp_.Process(x);
    const float h = hp_.High();
    const float b = bb0_ * h + bz1_;   // head bump
    bz1_ = bb1_ * h - ba1_ * b + bz2_;
    bz2_ = bb2_ * h - ba2_ * b;
    lp_.Process(b);
    hiss_lp_.Process(hiss * kHissLevel * noise_.Process());
    return lp_.Low() + hiss_lp_.Low();
}
