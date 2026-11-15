#pragma once
#include <cmath>

// Pad effect "crush" (P02 held, pressure = amount): digital degradation of the device output,
// after the tape sound: fewer bits (16 -> 4) and a lower sample rate (sample-and-hold,
// 48 kHz -> 2 kHz, aliasing on purpose). The amount is smoothed (~10 ms) and the crushed sound
// is blended in by it, so pressing and releasing don't click; at 0 the output is the input.
class Crush {
public:
    // Once per block: pad pressure 0..1. Also sets this block's bit depth and hold rate from the
    // smoothed amount (the blend weight stays smoothed per sample).
    void SetAmount(float a)
    {
        target_ = (a < 0.0f) ? 0.0f : (a > 1.0f ? 1.0f : a);
        rate_ = exp2f(-4.585f * amt_);           // hold rate 1 .. 2 kHz / 48 kHz
        q_ = exp2f(15.0f - 12.0f * amt_);        // 2^15 .. 2^3 steps per unit (16 .. 4 bits)
        inv_q_ = 1.0f / q_;
    }

    // Once per sample, both channels of the device output.
    void Process(float& l, float& r)
    {
        amt_ += 0.002f * (target_ - amt_);
        if (target_ == 0.0f && amt_ < 1e-4f) {   // idle
            amt_ = 0.0f;
            return;
        }
        // Sample rate: a new held value every 1 / rate samples.
        phase_ += rate_;
        if (phase_ >= 1.0f) {
            phase_ -= 1.0f;
            hold_l_ = l;
            hold_r_ = r;
        }
        // Bits.
        const float cl = roundf(hold_l_ * q_) * inv_q_;
        const float cr = roundf(hold_r_ * q_) * inv_q_;
        l += amt_ * (cl - l);
        r += amt_ * (cr - r);
    }

private:
    float target_ = 0.0f, amt_ = 0.0f;
    float phase_ = 1.0f;
    float hold_l_ = 0.0f, hold_r_ = 0.0f;
    float rate_ = 1.0f, q_ = 32768.0f, inv_q_ = 1.0f / 32768.0f;   // this block's hold rate and bit steps
};
