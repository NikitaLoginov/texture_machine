#include "tape_format.h"
#include <cmath>
using namespace daisysp;

void TapeFormat::Init(float sample_rate, int32_t noise_seed) {
    hp_.Init(sample_rate);
    lp_.Init(sample_rate);
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
    // Saturation drive: more for lo-fi formats and for a weak battery.
    drive_ = 1.5f + 4.5f * type + 6.0f * deficit;
    // Hiss: about -70 dB (reel) .. -40 dB (dictaphone), up to 4x louder with a weak battery.
    hiss_ = 0.0003f * powf(30.0f, type) * (1.0f + 3.0f * deficit);
}

float TapeFormat::Process(float in) {
    float x = tanhf(drive_ * in) / drive_;   // unity for quiet signals, saturates loud ones
    x += hiss_ * noise_.Process();
    hp_.Process(x);
    lp_.Process(hp_.High());
    return lp_.Low();
}
