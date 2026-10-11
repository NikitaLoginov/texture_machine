#include "grain_player.h"

#include <cmath>

void GrainPlayer::Init(const float* buf_l, const float* buf_r, size_t capacity, float sample_rate) {
    buf_l_ = buf_l;
    buf_r_ = buf_r;
    capacity_ = capacity;
    sr_ = sample_rate;

    length_ = 0;
    range_a_ = 0.0f;
    range_b_ = 1.0f;
    mode_ = FORWARD;
    speed_ = 1.0f;
    ratio_ = 1.0f;
    SetGrainMs(100.0f);
    SetFadeMs(10.0f);
    pos_ = 0.0f;
    dir_ = 1.0f;
    phase_ = 0.0f;
    start_[0] = 0.0f;
    start_[1] = 0.0f;
    gdir_[0] = 1.0f;
    gdir_[1] = 1.0f;

    UpdateRegion();
}

void GrainPlayer::SetLength(size_t length) {
    if (length > capacity_) length = capacity_;
    length_ = length;
    UpdateRegion();
}

void GrainPlayer::SetRange(float a, float b) {
    if (a < 0.0f) a = 0.0f; else if (a > 1.0f) a = 1.0f;
    if (b < 0.0f) b = 0.0f; else if (b > 1.0f) b = 1.0f;
    range_a_ = a;
    range_b_ = b;
    UpdateRegion();
}

void GrainPlayer::SetMode(Mode mode) {
    mode_ = mode;
    if (mode == FORWARD) dir_ = 1.0f;
    else if (mode == BACKWARD) dir_ = -1.0f;
    // PINGPONG keeps the current direction.
}

void GrainPlayer::SetSpeed(float speed) {
    speed_ = (speed < 0.0f) ? 0.0f : speed;
}

void GrainPlayer::SetPitch(float semitones) {
    if (semitones < -24.0f) semitones = -24.0f;
    else if (semitones > 24.0f) semitones = 24.0f;
    ratio_ = powf(2.0f, semitones / 12.0f);
}

void GrainPlayer::SetGrainMs(float ms) {
    if (ms < 5.0f) ms = 5.0f;
    else if (ms > 1000.0f) ms = 1000.0f;
    grain_len_ = ms * 0.001f * sr_;
    phase_inc_ = 1.0f / grain_len_;
}

void GrainPlayer::SetFadeMs(float ms) {
    if (ms < 1.0f) ms = 1.0f;
    else if (ms > 2000.0f) ms = 2000.0f;
    fade_len_ = ms * 0.001f * sr_;
}

void GrainPlayer::Reset() {
    pos_ = (mode_ == BACKWARD) ? (region_len_ - 1.0f) : 0.0f;
    if (pos_ < 0.0f) pos_ = 0.0f;
    phase_ = 0.0f;
    start_[0] = pos_;
    start_[1] = pos_;
    gdir_[0] = dir_;
    gdir_[1] = dir_;
}

void GrainPlayer::UpdateRegion() {
    const float lo = (range_a_ < range_b_) ? range_a_ : range_b_;
    const float hi = (range_a_ < range_b_) ? range_b_ : range_a_;
    const float len = (float)length_;
    region_start_ = lo * len;
    region_len_ = (hi - lo) * len;

    const float kMinRegion = 480.0f;   // 10 ms at 48 kHz
    if (region_len_ < kMinRegion) {
        region_len_ = kMinRegion;
        if (region_start_ + region_len_ > len) region_start_ = len - region_len_;
        if (region_start_ < 0.0f) {
            region_start_ = 0.0f;
            region_len_ = len;
        }
    }

    if (region_len_ >= 1.0f) pos_ = Wrap(pos_, region_len_);
    else pos_ = 0.0f;
}

float GrainPlayer::Wrap(float x, float len) {
    x = fmodf(x, len);
    if (x < 0.0f) x += len;
    if (x >= len) x = 0.0f;
    return x;
}

void GrainPlayer::Process(float& out_l, float& out_r) {
    static constexpr float kTwoPi = 6.28318530718f;

    if (length_ < 2 || region_len_ < 2.0f) {
        out_l = 0.0f;
        out_r = 0.0f;
        return;
    }

    // Advance playhead.
    if (mode_ == FORWARD) dir_ = 1.0f;
    else if (mode_ == BACKWARD) dir_ = -1.0f;
    pos_ += dir_ * speed_;
    if (mode_ == PINGPONG) {
        if (pos_ >= region_len_) { pos_ = 2.0f * region_len_ - pos_; dir_ = -1.0f; }
        if (pos_ < 0.0f) { pos_ = -pos_; dir_ = 1.0f; }
        if (pos_ < 0.0f || pos_ >= region_len_) pos_ = Wrap(pos_, region_len_);
    } else {
        pos_ = Wrap(pos_, region_len_);
    }

    // Advance grain phase; restart grain 0 at 1.0 and grain 1 at 0.5 (50% overlap).
    const float prev = phase_;
    phase_ += phase_inc_;
    if (phase_ >= 1.0f) {
        phase_ -= 1.0f;
        start_[0] = pos_;
        gdir_[0] = dir_;
    }
    if (prev < 0.5f && phase_ >= 0.5f) {
        start_[1] = pos_;
        gdir_[1] = dir_;
    }

    const float ph0 = phase_;
    const float ph1 = (phase_ >= 0.5f) ? (phase_ - 0.5f) : (phase_ + 0.5f);

    float l = 0.0f;
    float r = 0.0f;

    // Loop-edge fade: at most half the region, so fade-in and fade-out never overlap.
    float fade = fade_len_;
    if (fade > 0.5f * region_len_) fade = 0.5f * region_len_;
    const float inv_fade = 1.0f / fade;

    // Grain 0.
    {
        float rel = start_[0] + gdir_[0] * ph0 * grain_len_ * ratio_;
        rel = Wrap(rel, region_len_);
        float edge = rel * inv_fade;
        const float edge_end = (region_len_ - rel) * inv_fade;
        if (edge_end < edge) edge = edge_end;
        if (edge > 1.0f) edge = 1.0f;
        float idx = region_start_ + rel;
        size_t i0 = (size_t)idx;
        if (i0 >= length_) i0 = length_ - 1;
        const float frac = idx - (float)i0;
        size_t i1 = i0 + 1;
        if (i1 >= length_) i1 = 0;
        const float w = (0.5f - 0.5f * cosf(kTwoPi * ph0)) * edge;
        l += w * (buf_l_[i0] + frac * (buf_l_[i1] - buf_l_[i0]));
        r += w * (buf_r_[i0] + frac * (buf_r_[i1] - buf_r_[i0]));
    }

    // Grain 1.
    {
        float rel = start_[1] + gdir_[1] * ph1 * grain_len_ * ratio_;
        rel = Wrap(rel, region_len_);
        float edge = rel * inv_fade;
        const float edge_end = (region_len_ - rel) * inv_fade;
        if (edge_end < edge) edge = edge_end;
        if (edge > 1.0f) edge = 1.0f;
        float idx = region_start_ + rel;
        size_t i0 = (size_t)idx;
        if (i0 >= length_) i0 = length_ - 1;
        const float frac = idx - (float)i0;
        size_t i1 = i0 + 1;
        if (i1 >= length_) i1 = 0;
        const float w = (0.5f - 0.5f * cosf(kTwoPi * ph1)) * edge;
        l += w * (buf_l_[i0] + frac * (buf_l_[i1] - buf_l_[i0]));
        r += w * (buf_r_[i0] + frac * (buf_r_[i1] - buf_r_[i0]));
    }

    out_l = l;
    out_r = r;
}
