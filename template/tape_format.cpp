#include "tape_format.h"
#include <cmath>
using namespace daisysp;

// Steady hiss level: about -69 dBFS (hi-fi) .. -71 dBFS (lo-fi) RMS after its low-pass.
static constexpr float kHissLevel = 0.001f;

void TapeFormat::Init(float sample_rate, int32_t noise_seed) {
    hp_.Init(sample_rate);
    lp_.Init(sample_rate);
    hiss_lp_.Init(sample_rate);
    noise_.Init();
    noise_.SetSeed(noise_seed);
}

float TapeFormat::Wobble(float type) {
    return 0.0003f * powf(16.67f, type);  // 0.03 % (reel) .. 0.5 % (dictaphone)
}

void TapeFormat::SetParams(float type, float deficit, float speed, float res) {
    // Band edges per format, moving with tape speed: reel 25 Hz-16 kHz .. dictaphone 300 Hz-4 kHz.
    hp_.SetFreq(25.0f * powf(12.0f, type) * speed);
    hp_.SetRes(0.0f);
    lp_.SetFreq(16000.0f * powf(0.25f, type) * speed);
    lp_.SetRes(res);
    // Saturation drive 1 .. 4: more for lo-fi formats and for a weak battery.
    drive_ = 1.0f + 1.5f * type + 1.5f * deficit;
    // Make-up gain: a 0.5 peak comes out at 0.5 at any drive; quiet parts get up to +6 dB.
    norm_ = 0.5f / tanhf(0.5f * drive_);
    // Hiss low-pass: 12 kHz (hi-fi) .. 5 kHz (lo-fi). Not affected by speed or battery.
    hiss_lp_.SetFreq(12000.0f * powf(5.0f / 12.0f, type));
    hiss_lp_.SetRes(0.0074f);   // Q 0.707: flat up to the cutoff
}

float TapeFormat::Process(float in, float hiss) {
    const float x = tanhf(drive_ * in) * norm_;   // saturation with make-up gain
    hp_.Process(x);
    lp_.Process(hp_.High());
    hiss_lp_.Process(hiss * kHissLevel * noise_.Process());
    return lp_.Low() + hiss_lp_.Low();
}
