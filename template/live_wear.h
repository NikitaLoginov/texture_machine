#pragma once
#include <cstdint>
#include <cmath>

// S32 degradation on the live input through the tape, where nothing is written and there are no
// overdub passes: the live signal sounds as if it had been through 1 + 7 x S32 tape generations.
// Generation noise: the per-pass noise level x sqrt(generations) (noise of separate passes adds
// up in power), scaled like the aging low-pass would on tape, following the signal level (5 ms up,
// 50 ms down) over a quiet floor. Stutter: the tape's printed-stutter bursts (chance x
// generations) as pitch bumps on the grain speed, up and back, so the timing returns.
class LiveWear {
public:
    // Once per block, the S32 values: amount 0..1, the per-pass aging low-pass coefficient,
    // noise level, stutter bursts per sample and the peak pitch of a jerk (0.01 = 1 %).
    void SetAging(float amount, float lp_coef, float hiss, float burst_rate, float pitch)
    {
        const float gens = 1.0f + 7.0f * amount;
        const float a = (lp_coef > 0.0f) ? lp_coef : 0.0f;
        noise_ = hiss * sqrtf(gens) * sqrtf(a / (2.0f - a));   // white noise after that low-pass
        rate_ = burst_rate * gens;
        pitch_ = pitch;
    }

    // Once per block of n samples: factor on the tape speed for the live grains (1 = no jerk).
    float SpeedFactor(int n)
    {
        if (jerk_len_ == 0) {
            if (left_ > 0) {
                if (gap_ > 0) {
                    gap_ -= n;
                    return 1.0f;
                }
            } else {
                if (Rand01() >= rate_ * (float)n) return 1.0f;
                left_ = kBurstMin + (int32_t)(Rand01() * (float)(kBurstMax - kBurstMin + 1));
            }
            jerk_len_ = kJerkMin + (int32_t)(Rand01() * (float)(kJerkMax - kJerkMin));
            jerk_pos_ = 0;
            peak_ = pitch_ * Rand01();
            left_--;
            gap_ = (int32_t)(Rand01() * (float)kGapMax);
        }
        const float f = 1.0f + peak_ * sinf(6.28318530718f * (float)jerk_pos_ / (float)jerk_len_);
        jerk_pos_ += n;
        if (jerk_pos_ >= jerk_len_) jerk_len_ = 0;
        return f;
    }

    // Once per sample: adds the generation noise to both channels of the live signal.
    void Process(float& l, float& r)
    {
        const float lvl = 0.5f * (fabsf(l) + fabsf(r));
        env_ += ((lvl > env_) ? 0.004f : 0.0004f) * (lvl - env_);
        const float g = noise_ * (kNoiseFloor + kNoiseMod * env_);
        l += g * Noise();
        r += g * Noise();
    }

private:
    static constexpr int32_t kJerkMin = 1440;   // jerk length 30 .. 150 ms, samples (as on tape)
    static constexpr int32_t kJerkMax = 7200;
    static constexpr int32_t kGapMax = 2880;    // steady after each jerk of a burst, 0 .. 60 ms
    static constexpr int32_t kBurstMin = 2;     // jerks per burst
    static constexpr int32_t kBurstMax = 6;
    static constexpr float kNoiseFloor = 0.25f; // noise: floor + kNoiseMod x signal level (as on tape)
    static constexpr float kNoiseMod = 3.0f;

    float Noise()   // white noise -1 .. 1
    {
        seed_ ^= seed_ << 13;
        seed_ ^= seed_ >> 17;
        seed_ ^= seed_ << 5;
        return (float)(int32_t)seed_ * (1.0f / 2147483648.0f);
    }
    float Rand01() { return 0.5f + 0.5f * Noise(); }   // 0 .. 1

    uint32_t seed_ = 0x9E3779B9u;
    float   noise_ = 0.0f, rate_ = 0.0f, pitch_ = 0.0f;
    float   env_ = 0.0f;                  // signal level (envelope)
    int32_t jerk_len_ = 0, jerk_pos_ = 0; // current jerk (0 = none) and position, samples
    float   peak_ = 0.0f;                 // its peak pitch deviation
    int32_t left_ = 0, gap_ = 0;          // jerks left in the burst, steady samples before the next
};
