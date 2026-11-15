#pragma once
#include <cmath>

// Pad effect "phaser" (P05 held, pressure = depth + speed): a vintage 4-stage phaser on the
// device output (after the tape sound and tilt). Four first-order all-passes with feedback; their
// corner sweeps 200 Hz .. 2 kHz (log) with a sine LFO, 0.3 Hz (light) .. 2.4 Hz (hard). Depth =
// wet mix 0 .. 50 % (at 50 % dry and all-pass are equal: deep notches) + feedback 0 .. 0.5,
// smoothed (~10 ms); the LFO phase runs continuously; the all-pass coefficient is set per block.
// Released: exact bypass once the depth has faded, and it costs nothing. Checked in
// tools/sim/phaser_sim.py.
class Phaser {
public:
    void Init(float sample_rate) { sr_ = sample_rate; }

    // Once per block of n samples: pad pressure 0..1.
    void SetPressure(float p, int n)
    {
        target_ = (p < 0.0f) ? 0.0f : (p > 1.0f ? 1.0f : p);
        if (target_ == 0.0f && depth_ < 1e-4f) return;   // idle: nothing to sweep
        const float rate = 0.3f * exp2f(3.0f * target_);  // 0.3 .. 2.4 Hz
        ph_ += rate * (float)n / sr_;
        if (ph_ >= 1.0f) ph_ -= 1.0f;
        const float lfo = 0.5f - 0.5f * cosf(6.2831853f * ph_);   // 0 .. 1
        const float f = 200.0f * exp2f(3.3219281f * lfo);         // 200 Hz .. 2 kHz (x 10^lfo)
        const float t = tanf(3.14159265f * f / sr_);
        a_ = (t - 1.0f) / (t + 1.0f);
    }

    // Once per sample, both channels of the device output.
    void Process(float& l, float& r)
    {
        depth_ += 0.002f * (target_ - depth_);
        if (target_ == 0.0f && depth_ < 1e-4f) {   // idle
            depth_ = 0.0f;
            return;
        }
        l = Channel(0, l);
        r = Channel(1, r);
    }

private:
    float Channel(int c, float x)
    {
        float v = x + 0.5f * depth_ * fb_[c];
        for (int k = 0; k < 4; k++) {
            const float y = a_ * v + x1_[c][k] - a_ * y1_[c][k];
            x1_[c][k] = v;
            y1_[c][k] = y;
            v = y;
        }
        fb_[c] = v;
        return x + 0.5f * depth_ * (v - x);
    }

    float sr_ = 48000.0f;
    float target_ = 0.0f, depth_ = 0.0f;
    float ph_ = 0.0f;                 // LFO phase 0..1
    float a_ = 0.0f;                  // all-pass coefficient (this block)
    float x1_[2][4] = {}, y1_[2][4] = {};
    float fb_[2] = {0.0f, 0.0f};
};
