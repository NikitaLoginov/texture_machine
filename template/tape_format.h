#pragma once
#include "daisysp.h"

// Playback character of a tape format, one channel: saturation -> high-pass -> low-pass, + hiss.
class TapeFormat {
public:
    void  Init(float sample_rate, int32_t noise_seed);
    // Once per audio block. type: 0 = hi-fi reel-to-reel .. 1 = lo-fi dictaphone.
    // deficit: battery weakness 0..1. speed: tape speed factor (the band moves with it).
    // res: low-pass resonance 0..1 (Svf).
    void  SetParams(float type, float deficit, float speed, float res);
    float Process(float in);
    static float Wobble(float type);   // tape wow/flutter depth for Motor::SetWobble
private:
    daisysp::Svf        hp_, lp_, hiss_lp_;
    daisysp::WhiteNoise noise_;
    float drive_ = 1.0f, norm_ = 1.0f;     // saturation drive and make-up gain
};
