#include "grain_player.h"

#include <cmath>

void GrainPlayer::Init(const float* buf_l, const float* buf_r, size_t capacity, float sample_rate) {
    buf_l_ = buf_l;
    buf_r_ = buf_r;
    pre_l_ = nullptr;
    pre_r_ = nullptr;
    pre_len_ = 0;
    pre_phase_ = 0;
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
    pos_ = 0.0;
    dir_ = 1.0f;
    step_ = 0.0f;
    phase_ = 0.0f;
    start_[0] = 0.0;
    start_[1] = 0.0;
    gdir_[0] = 1.0f;
    gdir_[1] = 1.0f;
    off_[0] = off_[1] = 0.0;

    UpdateRegion();
    gbase_[0] = gbase_[1] = region_start_;
    glen_[0] = glen_[1] = region_len_;
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

void GrainPlayer::SetPreRoll(const float* l, const float* r, size_t len, size_t phase) {
    pre_l_ = l;
    pre_r_ = r;
    pre_len_ = len;
    pre_phase_ = (len > 0) ? phase % len : 0;
}

void GrainPlayer::Reset() {
    UpdateRegion();   // a pending loop change applies on play start
    pos_ = (mode_ == BACKWARD) ? (region_len_ - 1.0) : 0.0;
    if (pos_ < 0.0) pos_ = 0.0;
    phase_ = 0.0f;
    start_[0] = pos_;
    start_[1] = pos_;
    gdir_[0] = dir_;
    gdir_[1] = dir_;
    off_[0] = off_[1] = 0.0;
    gbase_[0] = gbase_[1] = region_start_;
    glen_[0] = glen_[1] = region_len_;
}

void GrainPlayer::UpdateRegion() {
    // Left fader (range_a_) sets the start, right fader (range_b_) the loop length as a share of
    // the recording after the start, so the end always follows the start.
    const double kMinRegion = 480.0;   // 10 ms at 48 kHz
    const double len = (double)length_;
    double start = floor(range_a_ * len);   // whole samples
    double rlen = floor(range_b_ * (len - start));
    if (rlen < kMinRegion) rlen = kMinRegion;
    if (start + rlen > len) start = len - rlen;
    if (start < 0.0) { start = 0.0; rlen = len; }
    region_start_ = start;
    region_len_ = rlen;

    if (region_len_ >= 1.0) pos_ = Wrap(pos_, region_len_);
    else pos_ = 0.0;
}

double GrainPlayer::Wrap(double x, double len) {
    x = fmod(x, len);
    if (x < 0.0) x += len;
    if (x >= len) x = 0.0;
    return x;
}

// Crossfade length (whole samples) for a loop of n samples starting at base: the S33 time,
// at most half the loop and at most the material before the start (recording + lead-in).
long GrainPlayer::Xfade(long base, long n) const {
    double x = fade_len_;
    if (x > 0.5 * (double)n) x = 0.5 * (double)n;
    const double avail = (double)base + (double)pre_len_;
    if (x > avail) x = avail;
    return (long)x;
}

// Loop sample k (0 .. n-1) of the loop starting at base. Over the last xf samples the loop end
// fades out while the audio leading into the start fades in (the recording before base, or the
// lead-in before the recording), so the wrap from n-1 back to 0 is seamless in both directions.
void GrainPlayer::LoopSample(long base, long n, long xf, long k, float& out_l, float& out_r) const {
    long i = base + k;
    if (i >= (long)length_) i = (long)length_ - 1;
    if (i < 0) i = 0;
    out_l = buf_l_[i];
    out_r = buf_r_[i];
    if (xf < 1 || k < n - xf) return;
    const float t = (float)(k - (n - xf)) / (float)xf;   // 0 .. 1 across the crossfade
    const long m = base - (n - k);                        // lead sample, before the start
    float lead_l = 0.0f, lead_r = 0.0f;
    if (m >= 0) {
        lead_l = buf_l_[m];
        lead_r = buf_r_[m];
    } else if (pre_len_ > 0 && -m <= (long)pre_len_) {
        const size_t j = (size_t)((long)pre_phase_ + (long)pre_len_ + m) % pre_len_;
        lead_l = pre_l_[j];
        lead_r = pre_r_[j];
    }
    out_l += t * (lead_l - out_l);
    out_r += t * (lead_r - out_r);
}

void GrainPlayer::Process(float& out_l, float& out_r) {
    static constexpr float kTwoPi = 6.28318530718f;

    if (length_ < 2 || region_len_ < 2.0) {
        step_ = 0.0f;
        out_l = 0.0f;
        out_r = 0.0f;
        return;
    }

    // Advance playhead.
    if (mode_ == FORWARD) dir_ = 1.0f;
    else if (mode_ == BACKWARD) dir_ = -1.0f;
    step_ = dir_ * speed_;
    pos_ += step_;
    // A pass ends at a wrap or bounce: a pending loop change (SetRange) applies there, so the
    // playhead continues from the new start or end instead of jumping in the middle of a pass.
    if (mode_ == PINGPONG) {
        if (pos_ >= region_len_) {
            const double over = pos_ - region_len_;
            UpdateRegion();
            pos_ = region_len_ - over;
            dir_ = -1.0f;
        }
        if (pos_ < 0.0) {
            const double over = -pos_;
            UpdateRegion();
            pos_ = over;
            dir_ = 1.0f;
        }
        if (pos_ < 0.0 || pos_ >= region_len_) pos_ = Wrap(pos_, region_len_);
    } else if (pos_ >= region_len_ || pos_ < 0.0) {
        const double over = (pos_ >= region_len_) ? pos_ - region_len_ : pos_;   // backward: < 0
        UpdateRegion();
        pos_ = Wrap(over, region_len_);
    }

    // Advance grain phase; restart grain 0 at 1.0 and grain 1 at 0.5 (50% overlap).
    const float prev = phase_;
    phase_ += phase_inc_;
    if (phase_ >= 1.0f) {
        phase_ -= 1.0f;
        start_[0] = pos_;
        gdir_[0] = dir_;
        off_[0] = 0.0;
        gbase_[0] = region_start_;
        glen_[0] = region_len_;
    }
    if (prev < 0.5f && phase_ >= 0.5f) {
        start_[1] = pos_;
        gdir_[1] = dir_;
        off_[1] = 0.0;
        gbase_[1] = region_start_;
        glen_[1] = region_len_;
    }

    const float ph[2] = {phase_, (phase_ >= 0.5f) ? (phase_ - 0.5f) : (phase_ + 0.5f)};

    float l = 0.0f;
    float r = 0.0f;

    // Each grain reads the region it started in, so a region change never makes a sounding
    // grain jump: old grains fade out in the old region, new ones fade in in the new region.
    for (int g = 0; g < 2; g++) {
        const double len = glen_[g];
        if (len < 2.0) continue;   // grain started before there was a valid region
        const long base = (long)gbase_[g];
        const long n = (long)len;
        const long xf = Xfade(base, n);
        const double rel = Wrap(start_[g] + off_[g], len);
        long k0 = (long)rel;
        if (k0 >= n) k0 = n - 1;
        const float frac = (float)(rel - (double)k0);
        long k1 = k0 + 1;
        if (k1 >= n) k1 = 0;   // the loop end runs into its start
        float l0, r0, l1, r1;
        LoopSample(base, n, xf, k0, l0, r0);
        LoopSample(base, n, xf, k1, l1, r1);
        const float w = 0.5f - 0.5f * cosf(kTwoPi * ph[g]);
        l += w * (l0 + frac * (l1 - l0));
        r += w * (r0 + frac * (r1 - r0));
    }

    // Advance grain read offsets (integrated, so ratio changes don't make grains jump).
    off_[0] += gdir_[0] * ratio_;
    off_[1] += gdir_[1] * ratio_;

    out_l = l;
    out_r = r;
}
