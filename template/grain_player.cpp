#include "grain_player.h"

#include <cmath>

void GrainPlayer::Init(const float* buf_l, const float* buf_r, size_t capacity, float sample_rate) {
    buf_l_ = buf_l;
    buf_r_ = buf_r;
    pre_l_ = nullptr;
    pre_r_ = nullptr;
    pre_len_ = 0;
    pre_phase_ = 0;
    ovd_l_ = nullptr;
    ovd_r_ = nullptr;
    ovd_pre_l_ = nullptr;
    ovd_pre_r_ = nullptr;
    ovd_pre_len_ = 0;
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
    next_start_ = next_len_ = 0.0;

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
}

void GrainPlayer::SetMode(Mode mode) {
    if (mode != mode_) {   // a loop change waiting for the wrap starts over in the new mode
        latched_ = false;
        lead_start_ = region_start_;
    }
    mode_ = mode;
    if (mode == FORWARD) dir_ = 1.0f;
    else if (mode == BACKWARD) dir_ = -1.0f;
    // PINGPONG keeps the current direction.
}

void GrainPlayer::SetSpeed(float speed) {
    speed_ = (speed < 0.0f) ? 0.0f : speed;
}

void GrainPlayer::SetPitch(float semitones) {
    if (semitones < -36.0f) semitones = -36.0f;
    else if (semitones > 36.0f) semitones = 36.0f;
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

void GrainPlayer::SetOverdub(const float* l, const float* r, const float* pre_l, const float* pre_r, size_t pre_len) {
    ovd_l_ = l;
    ovd_r_ = r;
    ovd_pre_l_ = pre_l;
    ovd_pre_r_ = pre_r;
    ovd_pre_len_ = pre_len;
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
}

// Applies the loop set by SetRange now (new recording, play start).
void GrainPlayer::UpdateRegion() {
    NextRegion(region_start_, region_len_);
    lead_start_ = region_start_;
    latched_ = false;
    if (region_len_ >= 1.0) pos_ = Wrap(pos_, region_len_);
    else pos_ = 0.0;
}

// The loop set by SetRange. Left fader (range_a_) sets the start, right fader (range_b_) the loop
// length as a share of the recording after the start, so the end always follows the start.
void GrainPlayer::NextRegion(double& start, double& rlen) const {
    const double kMinRegion = 480.0;   // 10 ms at 48 kHz
    const double len = (double)length_;
    start = floor(range_a_ * len);   // whole samples
    rlen = floor(range_b_ * (len - start));
    if (rlen < kMinRegion) rlen = kMinRegion;
    if (start + rlen > len) start = len - rlen;
    if (start < 0.0) { start = 0.0; rlen = len; }
}

double GrainPlayer::Wrap(double x, double len) {
    x = fmod(x, len);
    if (x < 0.0) x += len;
    if (x >= len) x = 0.0;
    return x;
}

// Crossfade length (whole samples) for a loop of n samples: the S33 time, at most half the loop.
// (The lead-in is as long as the longest crossfade, so there is always material before a start.)
long GrainPlayer::Xfade(long n) const {
    double x = fade_len_;
    if (x > 0.5 * (double)n) x = 0.5 * (double)n;
    return (long)x;
}

// Overdub track seam (whole samples): kOvdXfade, at most half the loop.
long GrainPlayer::OvdXfade(long n) const {
    return (kOvdXfade < n / 2) ? kOvdXfade : n / 2;
}

// Sample m of the recording, or of the lead-in when m < 0 (silence when out of range).
void GrainPlayer::Lead(long m, float& out_l, float& out_r) const {
    out_l = 0.0f;
    out_r = 0.0f;
    if (m >= 0) {
        if (m < (long)length_) {
            out_l = buf_l_[m];
            out_r = buf_r_[m];
        }
    } else if (pre_len_ > 0 && -m <= (long)pre_len_) {
        const size_t j = (size_t)((long)pre_phase_ + (long)pre_len_ + m) % pre_len_;
        out_l = pre_l_[j];
        out_r = pre_r_[j];
    }
}

// Overdub track sample m, or its lead-in when m < 0 (silence when out of range).
void GrainPlayer::OvdLead(long m, float& out_l, float& out_r) const {
    out_l = 0.0f;
    out_r = 0.0f;
    if (m >= 0) {
        if (m < (long)length_) {
            out_l = ovd_l_[m];
            out_r = ovd_r_[m];
        }
    } else if (-m <= (long)ovd_pre_len_) {
        out_l = ovd_pre_l_[(long)ovd_pre_len_ + m];
        out_r = ovd_pre_r_[(long)ovd_pre_len_ + m];
    }
}

// Loop sample k (0 .. n) of the current loop of n samples: recording + overdub track.
// Recording: over the last xf samples (S33) an equal-power crossfade from the loop end into the
// audio before lead_start_. Overdub track: the same over its own short seam xo, into the overdub
// track before lead_start_. k = n is the sample that follows the end: the leads at distance 0.
// Normally lead_start_ is the loop start; during a loop change it is the other loop's start.
void GrainPlayer::LoopSample(long n, long xf, long xo, long k, float& out_l, float& out_r) const {
    long i = (long)region_start_ + ((k >= n) ? 0 : k);
    if (i >= (long)length_) i = (long)length_ - 1;
    if (i < 0) i = 0;
    // Recording.
    if (k >= n && xf >= 1) {
        Lead((long)lead_start_, out_l, out_r);
    } else {
        out_l = buf_l_[i];
        out_r = buf_r_[i];
        if (xf >= 1 && k >= n - xf) {
            const float u = 1.5707963f * (float)(k - (n - xf)) / (float)xf;
            const float c = cosf(u), s = sinf(u);
            float ll, lr;
            Lead((long)lead_start_ - (n - k), ll, lr);
            out_l = c * out_l + s * ll;
            out_r = c * out_r + s * lr;
        }
    }
    // Overdub track.
    if (ovd_l_ == nullptr) return;
    float ol, orr;
    if (k >= n && xo >= 1) {
        OvdLead((long)lead_start_, ol, orr);
    } else {
        ol = ovd_l_[i];
        orr = ovd_r_[i];
        if (xo >= 1 && k >= n - xo) {
            const float u = 1.5707963f * (float)(k - (n - xo)) / (float)xo;
            const float c = cosf(u), s = sinf(u);
            float ll, lr;
            OvdLead((long)lead_start_ - (n - k), ll, lr);
            ol = c * ol + s * ll;
            orr = c * orr + s * lr;
        }
    }
    out_l += ol;
    out_r += orr;
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

    // A loop change (SetRange) goes through the loop crossfade, like a normal wrap, and all grains
    // move with the playhead into the new loop:
    // Forward: fixed kLeadMargin before the playhead enters the crossfade, whose lead then comes from before the
    //   NEW start; applied at the wrap.
    // Backward, Ping-Pong (end bounce only): applied at the wrap / bounce; the new loop's first
    //   crossfade keeps the lead from before the OLD start, until the playhead leaves it.
    double ns, nl;
    if (mode_ == FORWARD) {
        if (!latched_ && pos_ >= region_len_ - (double)Xfade((long)region_len_) - kLeadMargin) {
            NextRegion(ns, nl);
            if (ns != region_start_ || nl != region_len_) {
                latched_ = true;
                next_start_ = ns;
                next_len_ = nl;
                lead_start_ = ns;
            }
        }
        if (pos_ >= region_len_) {
            const double over = pos_ - region_len_;
            if (latched_) {
                for (int g = 0; g < 2; g++) {
                    start_[g] += off_[g] - region_len_;
                    off_[g] = 0.0;
                }
                region_start_ = next_start_;
                region_len_ = next_len_;
                lead_start_ = region_start_;
                latched_ = false;
            }
            pos_ = Wrap(over, region_len_);
        }
    } else if (mode_ == BACKWARD) {
        if (pos_ < 0.0) {
            NextRegion(ns, nl);
            if (ns != region_start_ || nl != region_len_) {
                for (int g = 0; g < 2; g++) {
                    start_[g] += off_[g] + nl;
                    off_[g] = 0.0;
                }
                lead_start_ = region_start_;
                region_start_ = ns;
                region_len_ = nl;
            }
            pos_ = Wrap(pos_, region_len_);
        }
        if (lead_start_ != region_start_ && pos_ < region_len_ - (double)Xfade((long)region_len_) - kLeadMargin)
            lead_start_ = region_start_;
    } else {
        if (pos_ >= region_len_) {
            const double over = pos_ - region_len_;
            NextRegion(ns, nl);
            if (ns != region_start_ || nl != region_len_) {
                // Grains turn with the playhead (mirrored at the old end) and keep their distance
                // to the end.
                for (int g = 0; g < 2; g++) {
                    double u = start_[g] + off_[g];
                    if (gdir_[g] > 0.0f) u = 2.0 * region_len_ - u;
                    start_[g] = u + (nl - region_len_);
                    off_[g] = 0.0;
                    gdir_[g] = -1.0f;
                }
                lead_start_ = region_start_;
                region_start_ = ns;
                region_len_ = nl;
            }
            pos_ = region_len_ - over;
            dir_ = -1.0f;
        }
        if (pos_ < 0.0) { pos_ = -pos_; dir_ = 1.0f; }
        // Bounce stays inside the loop (a position exactly on the end must not wrap to 0).
        if (pos_ >= region_len_) pos_ = region_len_ - 0.000001;
        if (pos_ < 0.0) pos_ = 0.0;
        if (lead_start_ != region_start_ && dir_ < 0.0f &&
            pos_ < region_len_ - (double)Xfade((long)region_len_) - kLeadMargin)
            lead_start_ = region_start_;
    }

    // Advance grain phase; restart grain 0 at 1.0 and grain 1 at 0.5 (50% overlap).
    const float prev = phase_;
    phase_ += phase_inc_;
    if (phase_ >= 1.0f) {
        phase_ -= 1.0f;
        start_[0] = pos_;
        gdir_[0] = dir_;
        off_[0] = 0.0;
    }
    if (prev < 0.5f && phase_ >= 0.5f) {
        start_[1] = pos_;
        gdir_[1] = dir_;
        off_[1] = 0.0;
    }

    const float ph[2] = {phase_, (phase_ >= 0.5f) ? (phase_ - 0.5f) : (phase_ + 0.5f)};

    float l = 0.0f;
    float r = 0.0f;

    const long n = (long)region_len_;
    const long xf = Xfade(n);
    const long xo = OvdXfade(n);
    for (int g = 0; g < 2; g++) {
        const double rel = Wrap(start_[g] + off_[g], region_len_);
        long k0 = (long)rel;
        if (k0 >= n) k0 = n - 1;
        const float frac = (float)(rel - (double)k0);
        float l0, r0, l1, r1;
        LoopSample(n, xf, xo, k0, l0, r0);
        LoopSample(n, xf, xo, k0 + 1, l1, r1);   // k0 + 1 = n: what follows the end (the lead)
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
