#pragma once
#include "daisysp.h"

// Playback character of a tape format, one channel: saturation -> high-pass -> low-pass, + hiss.
class TapeFormat {
public:
    void  Init(float sample_rate, int32_t noise_seed);
    // Once per audio block. type: 0 = hi-fi reel-to-reel .. 1 = lo-fi dictaphone.
    // deficit: battery weakness 0..1. speed: tape speed factor (the band moves with it).
    // drive: multiplier on the saturation drive (S35): 0.5 (cleaner) .. 1 (as set by type and
    // battery) .. 4 (much harder). bump: head bump amount 0 .. 1 (0 .. +9 dB).
    void  SetParams(float type, float deficit, float speed, float drive, float bump);
    float Process(float in, float hiss = 1.0f);   // hiss: 0..1 gain on the tape hiss
    static float Wobble(float type);   // tape wow/flutter depth for Motor::SetWobble
private:
    daisysp::Svf        hp_, lp_, hiss_lp_;
    daisysp::WhiteNoise noise_;
    float drive_ = 1.0f, norm_ = 1.0f;     // saturation drive and make-up gain
    float sr_ = 48000.0f;
    // Head bump: peaking biquad (transposed direct form II), coefficients per block.
    float bb0_ = 1.0f, bb1_ = 0.0f, bb2_ = 0.0f, ba1_ = 0.0f, ba2_ = 0.0f;
    float bz1_ = 0.0f, bz2_ = 0.0f;
};
