#pragma once
#include <cstddef>
#include <cmath>

// Live input through the tape: the input goes into a ring and two Hann grains (50 % overlap)
// read it at the tape speed (ratio), so the pitch follows the tape like a playing tape while the
// tempo stays live. Each grain starts a delay behind the newest input: 10 ms, plus enough that a
// grain running faster than the input (ratio > 1) does not reach it within its life. If the tape
// speeds up during a grain, the grain's read stops at the input and follows it (its pitch bends
// toward 1, no jump). Checked in tools/sim/live_input_sim.py.
class LiveGrains {
public:
    // ring_l / ring_r: size samples each (1 s or more), cleared here.
    void Init(float* ring_l, float* ring_r, size_t size, float sample_rate)
    {
        ring_l_ = ring_l;
        ring_r_ = ring_r;
        size_ = size;
        for (size_t i = 0; i < size_; i++) ring_l_[i] = ring_r_[i] = 0.0f;
        grain_len_ = 0.1f * sample_rate;   // 100 ms, like the tape grains
        min_delay_ = 0.01f * sample_rate;  // 10 ms
        wi_ = 0;
        ratio_ = 1.0f;
        Reset();
    }

    // Tape speed, once per block. Clamped to 0 .. 4.
    void SetRatio(float ratio)
    {
        ratio_ = (ratio < 0.0f) ? 0.0f : (ratio > 4.0f ? 4.0f : ratio);
    }

    // Both grains start again at the delay behind the newest input (call when the mode turns on).
    void Reset()
    {
        phase_ = 0.0f;
        back_[0] = back_[1] = Delay();
    }

    // One stereo frame: writes the input, returns the grains' output.
    void Process(float in_l, float in_r, float& out_l, float& out_r)
    {
        if (++wi_ >= size_) wi_ = 0;
        ring_l_[wi_] = in_l;
        ring_r_[wi_] = in_r;
        // Grain 0 restarts at phase 1, grain 1 at phase 0.5.
        const float prev = phase_;
        phase_ += 1.0f / grain_len_;
        if (phase_ >= 1.0f) {
            phase_ -= 1.0f;
            back_[0] = Delay();
        }
        if (prev < 0.5f && phase_ >= 0.5f) back_[1] = Delay();
        float l0, r0, l1, r1;
        Read(back_[0], l0, r0);
        Read(back_[1], l1, r1);
        const float w0 = 0.5f - 0.5f * cosf(6.28318530718f * phase_);   // grain 1: 1 - w0
        out_l = l1 + w0 * (l0 - l1);
        out_r = r1 + w0 * (r0 - r1);
        // Next sample: the input moves 1 ahead, each read moves ratio ahead.
        for (int g = 0; g < 2; g++) {
            back_[g] += 1.0f - ratio_;
            if (back_[g] < kGap) back_[g] = kGap;   // never read past the input
            const float most = (float)size_ - 2.0f;
            if (back_[g] > most) back_[g] = most;
        }
    }

private:
    static constexpr float kGap = 2.0f;   // a read stays at least this far behind the newest input

    // Delay of a starting grain: 10 ms + what a grain at this ratio gains on the input in its life.
    float Delay() const
    {
        const float gain = (ratio_ > 1.0f) ? grain_len_ * (ratio_ - 1.0f) : 0.0f;
        return min_delay_ + gain;
    }

    // Both channels back samples behind the newest input, interpolated.
    void Read(float back, float& out_l, float& out_r) const
    {
        const size_t whole = (size_t)back;
        const float frac = back - (float)whole;
        // Sample i0 is `whole` behind the newest, i1 one more behind; the read is between them.
        size_t i0 = (wi_ >= whole) ? wi_ - whole : wi_ + size_ - whole;
        size_t i1 = (i0 > 0) ? i0 - 1 : size_ - 1;
        out_l = ring_l_[i0] + frac * (ring_l_[i1] - ring_l_[i0]);
        out_r = ring_r_[i0] + frac * (ring_r_[i1] - ring_r_[i0]);
    }

    float* ring_l_ = nullptr;
    float* ring_r_ = nullptr;
    size_t size_ = 0;
    size_t wi_ = 0;              // ring index of the newest input sample
    float  grain_len_ = 4800.0f; // samples
    float  min_delay_ = 480.0f;  // samples
    float  ratio_ = 1.0f;
    float  phase_ = 0.0f;
    float  back_[2] = {0.0f, 0.0f};   // per grain: read position, samples behind the newest input
};
