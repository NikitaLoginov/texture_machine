#pragma once
#include <cmath>

// One-knob tilt EQ, one channel: pivots around 1 kHz, up to about +-6 dB at the ends of the
// spectrum. A one-pole crossover splits lows and highs; the crossover frequency is the pivot times
// the high gain, which keeps the pivot unchanged at any setting (tools/sim/tilt_check.py).
class TiltEq {
public:
    void Init(float sample_rate)
    {
        sr_ = sample_rate;
        SetTilt(0.0f);
    }

    // tilt: -1 (darker: lows +6 dB, highs -6 dB) .. 0 (flat) .. 1 (brighter). Once per block.
    void SetTilt(float tilt)
    {
        const float g = powf(10.0f, 6.0f * tilt / 20.0f);
        gh_ = g;
        gl_ = 1.0f / g;
        a_ = 1.0f - expf(-6.2831853f * 1000.0f * g / sr_);
    }

    float Process(float x)
    {
        lp_ += a_ * (x - lp_);
        return gl_ * lp_ + gh_ * (x - lp_);
    }

private:
    float sr_ = 48000.0f;
    float a_ = 0.1f;               // crossover one-pole coefficient
    float gl_ = 1.0f, gh_ = 1.0f;  // low and high gains
    float lp_ = 0.0f;
};
