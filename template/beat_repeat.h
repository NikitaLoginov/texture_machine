#pragma once
#include <cstddef>
#include <cstdint>
#include "fast_math.h"

// Pad effect "beat repeat" (P07 held): the tape signal (before the tape sound, so hiss and the
// tape's filters stay live) is recorded into a ring all the time. Pressing starts a slice at that
// moment; its first pass is the live signal itself (no crossfade needed), then it repeats. The
// last kXf samples of each pass crossfade (equal power) into the material just before the slice
// start, then the read wraps. A new slice length applies at the next wrap, so the rhythm holds.
// Release: 10 ms equal-power crossfade back to the running tape (at once during the first pass).
// After a press the ring records only until it is full, so the slice is never overwritten while
// it repeats. Checked in tools/sim/beat_repeat_sim.py.
class BeatRepeat {
public:
    // ring_l / ring_r: size samples each (the longest slice + 2 kXf + 16 or more), cleared here.
    void Init(float* ring_l, float* ring_r, size_t size)
    {
        ring_l_ = ring_l;
        ring_r_ = ring_r;
        size_ = size;
        for (size_t i = 0; i < size_; i++) ring_l_[i] = ring_r_[i] = 0.0f;
        max_len_ = (int32_t)size_ - 2 * kXf - 16;
    }

    // Once per block: held, and the slice length in samples (taken at the press and at each wrap).
    void Set(bool hold, float length)
    {
        hold_ = hold;
        int32_t n = (int32_t)length;
        if (n < 2 * kXf) n = 2 * kXf;
        if (n > max_len_) n = max_len_;
        next_len_ = n;
    }

    // Once per sample, both channels of the tape signal, in place.
    void Process(float& l, float& r)
    {
        bool idle = !active_ && g_ == 0.0f;
        if (hold_ && idle) {
            // The first pass is the live signal itself: switch at once (an equal-power fade of
            // two identical signals would bump the level).
            active_ = true;
            first_ = true;
            g_ = 1.0f;
            start_ = wi_;
            p_ = 0;
            len_ = next_len_;
            rec_ = 0;
            idle = false;
        }
        // Record while idle; after a press only until the ring is full.
        if (idle || rec_ < (int32_t)size_ - kXf - 16) {
            ring_l_[wi_] = l;
            ring_r_[wi_] = r;
            if (++wi_ >= size_) wi_ = 0;
            if (!idle) rec_++;
        }
        if (!hold_) {
            if (active_ && first_) g_ = 0.0f;   // released during the first pass: back at once
            active_ = false;
        }
        if (!active_ && g_ == 0.0f) return;
        // Slice read; its last kXf samples crossfade into the lead just before the slice start.
        size_t i = start_ + (size_t)p_;
        if (i >= size_) i -= size_;
        float vl = ring_l_[i], vr = ring_r_[i];
        const int32_t k = p_ - (len_ - kXf);
        if (k >= 0) {
            const float u = (float)k / (float)kXf;
            const float c = FastSinHalfPi(1.0f - u), s = FastSinHalfPi(u);
            size_t j = start_ + size_ - (size_t)kXf + (size_t)k;
            if (j >= size_) j -= size_;
            vl = c * vl + s * ring_l_[j];
            vr = c * vr + s * ring_r_[j];
        }
        if (++p_ >= len_) {
            p_ = 0;
            len_ = next_len_;   // a new length (division) from the next pass on
            first_ = false;
        }
        g_ = active_ ? fminf(g_ + kRel, 1.0f) : fmaxf(g_ - kRel, 0.0f);
        const float w = FastSinHalfPi(g_), c = FastSinHalfPi(1.0f - g_);
        l = c * l + w * vl;
        r = c * r + w * vr;
    }

private:
    static constexpr int32_t kXf = 288;           // slice wrap crossfade, 6 ms
    static constexpr float kRel = 1.0f / 480.0f;  // release crossfade step, 10 ms

    float* ring_l_ = nullptr;
    float* ring_r_ = nullptr;
    size_t size_ = 0;
    size_t wi_ = 0;          // next ring write index
    size_t start_ = 0;       // ring index of the slice start
    int32_t p_ = 0;          // read position in the slice
    int32_t len_ = 0;        // current slice length
    int32_t next_len_ = 0;   // length for the next pass
    int32_t max_len_ = 0;
    int32_t rec_ = 0;        // samples recorded since the press
    bool hold_ = false, active_ = false, first_ = false;
    float g_ = 0.0f;         // repeat share, 0 = live tape .. 1 = repeat
};
