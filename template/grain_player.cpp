#include "grain_player.h"
#include "fast_math.h"
#include "debug_trace.h"

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
    writing_ = false;
    mode_ = FORWARD;
    speed_ = 1.0f;
    ratio_ = 1.0f;
    SetGrainMs(100.0f);
    SetFadeMs(10.0f);
    pos_ = 0.0;
    dir_ = 1.0f;
    step_ = 0.0f;
    phase_ = 0.0f;

    UpdateRegion();
    for (int g = 0; g < 2; g++) {
        start_[g] = 0.0;
        gdir_[g] = 1.0f;
        off_[g] = 0.0;
        gbase_[g] = region_start_;
        glen_[g] = region_len_;
    }
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

void GrainPlayer::SetRatio(float ratio) {
    if (ratio < 0.125f) ratio = 0.125f;   // -36 semitones
    else if (ratio > 8.0f) ratio = 8.0f;  // +36 semitones
    ratio_ = ratio;
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
    for (int g = 0; g < 2; g++) {
        start_[g] = pos_;
        gdir_[g] = dir_;
        off_[g] = 0.0;
        gbase_[g] = region_start_;
        glen_[g] = region_len_;
    }
}

// Applies the loop set by SetRange now (new recording, play start). Playing grains keep their loop.
void GrainPlayer::UpdateRegion() {
    NextRegion(region_start_, region_len_);
    if (region_len_ >= 1.0) pos_ = Wrap(pos_, region_len_);
    else pos_ = 0.0;
}

// The loop set by SetRange. Left fader (range_a_) sets the start, right fader (range_b_) the loop
// length out of the recording after the start, so the end always follows the start. The length
// is exponential: 100 ms (one grain, shorter loops buzz) at 0 .. all of the rest at 1, the same
// fader travel for each doubling.
void GrainPlayer::NextRegion(double& start, double& rlen) const {
    const double kMinRegion = 0.1 * (double)sr_;   // 100 ms
    const double len = (double)length_;
    start = floor(range_a_ * len);   // whole samples
    const double rest = len - start;
    rlen = (rest > kMinRegion) ? floor(kMinRegion * (double)powf((float)(rest / kMinRegion), range_b_)) : kMinRegion;
    if (rlen < kMinRegion) rlen = kMinRegion;
    if (start + rlen > len) start = len - rlen;
    if (start < 0.0) { start = 0.0; rlen = len; }
}

// While the record head writes, the loop holds still with the playhead in the end crossfade zone
// (where the head writes the seam and the lead before the start) or at the very start (its tails
// past the end), each plus kEdgeHold. A loop too short to leave room for a grain start between
// these zones is never held (the faders would stop working).
bool GrainPlayer::Frozen() const {
    if (!writing_) return false;
    const double n = region_len_;
    long xz = Xfade((long)n);
    const long xo = OvdXfade((long)n);
    if (xo > xz) xz = xo;
    const double room = n - (double)xz - 2.0 * kEdgeHold;
    if (room < (double)speed_ * (double)grain_len_) return false;   // two grain starts' travel
    return pos_ >= n - (double)xz - kEdgeHold || pos_ < kEdgeHold;
}

// Fader changes take effect at a grain start (every half grain): the playhead keeps its place in
// the loop and the new grain reads the new loop, while the other grain finishes in the old one.
void GrainPlayer::Apply() {
    if (Frozen()) return;
    double ns, nl;
    NextRegion(ns, nl);
    if (ns == region_start_ && nl == region_len_) return;
    region_start_ = ns;
    region_len_ = nl;
    pos_ = Wrap(pos_, region_len_);
}

void GrainPlayer::StartGrain(int g) {
    TRACE_STAGE(kTrGrainStart);
    Apply();
    TRACE_SET(rs, (int32_t)region_start_);
    TRACE_SET(rl, (int32_t)region_len_);
    TRACE_SET(pos, (float)pos_);
    start_[g] = pos_;
    gdir_[g] = dir_;
    off_[g] = 0.0;
    gbase_[g] = region_start_;
    glen_[g] = region_len_;
}

double GrainPlayer::Wrap(double x, double len) {
    // Positions are almost always within one loop length of the loop: subtract instead of fmod
    // (same result, the subtraction is exact).
    if (x >= len) {
        x -= len;
        if (x >= len) x = fmod(x, len);
    } else if (x < 0.0) {
        x += len;
        if (x < 0.0) {
            x = fmod(x, len);
            if (x < 0.0) x += len;
        }
    }
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

// Loop sample k (0 .. n) of the loop of n samples starting at base: recording + overdub track.
// Recording: over the last xf samples (S33) an equal-power crossfade from the loop end into the
// audio before base. Overdub track: the same over its own short seam xo, into the overdub track
// before base. k = n is the sample that follows the end: the leads at distance 0.
void GrainPlayer::LoopSample(long base, long n, long xf, long xo, long k, float& out_l, float& out_r) const {
    long i = base + ((k >= n) ? 0 : k);
    if (i >= (long)length_) i = (long)length_ - 1;
    if (i < 0) i = 0;
    // Recording.
    if (k >= n && xf >= 1) {
        Lead(base, out_l, out_r);
    } else {
        out_l = buf_l_[i];
        out_r = buf_r_[i];
        if (xf >= 1 && k >= n - xf) {
            const float u = (float)(k - (n - xf)) / (float)xf;
            const float c = FastSinHalfPi(1.0f - u), s = FastSinHalfPi(u);
            float ll, lr;
            Lead(base - (n - k), ll, lr);
            out_l = c * out_l + s * ll;
            out_r = c * out_r + s * lr;
        }
    }
    // Overdub track.
    if (ovd_l_ == nullptr) return;
    float ol, orr;
    if (k >= n && xo >= 1) {
        OvdLead(base, ol, orr);
    } else {
        ol = ovd_l_[i];
        orr = ovd_r_[i];
        if (xo >= 1 && k >= n - xo) {
            const float u = (float)(k - (n - xo)) / (float)xo;
            const float c = FastSinHalfPi(1.0f - u), s = FastSinHalfPi(u);
            float ll, lr;
            OvdLead(base - (n - k), ll, lr);
            ol = c * ol + s * ll;
            orr = c * orr + s * lr;
        }
    }
    out_l += ol;
    out_r += orr;
}

// Both tracks at position rel (0 .. n) of the loop of n samples starting at base, interpolated
// between loop samples.
void GrainPlayer::ReadAt(double rel, long base, long n, float& out_l, float& out_r) const {
    const long xf = Xfade(n);
    const long xo = OvdXfade(n);
    long k0 = (long)rel;
    if (k0 >= n) k0 = n - 1;
    const float frac = (float)(rel - (double)k0);
    float l0, r0, l1, r1;
    LoopSample(base, n, xf, xo, k0, l0, r0);
    LoopSample(base, n, xf, xo, k0 + 1, l1, r1);   // k0 + 1 = n: what follows the end (the lead)
    out_l = l0 + frac * (l1 - l0);
    out_r = r0 + frac * (r1 - r0);
}

void GrainPlayer::Process(float& out_l, float& out_r) {
    static constexpr float kTwoPi = 6.28318530718f;

    TRACE_STAGE(kTrPlayer);
    if (length_ < 2 || region_len_ < 2.0) {
        step_ = 0.0f;
        out_l = 0.0f;
        out_r = 0.0f;
        return;
    }

    // Advance playhead: wrap at the loop edges, or bounce in ping-pong.
    if (mode_ == FORWARD) dir_ = 1.0f;
    else if (mode_ == BACKWARD) dir_ = -1.0f;
    step_ = dir_ * speed_;
    pos_ += step_;
    if (mode_ == PINGPONG) {
        if (pos_ >= region_len_) { pos_ = 2.0 * region_len_ - pos_; dir_ = -1.0f; }
        if (pos_ < 0.0) { pos_ = -pos_; dir_ = 1.0f; }
        // Bounce stays inside the loop (a position exactly on the end must not wrap to 0).
        if (pos_ >= region_len_) pos_ = region_len_ - 0.000001;
        if (pos_ < 0.0) pos_ = 0.0;
    } else {
        pos_ = Wrap(pos_, region_len_);
    }
    TRACE_SET(mode, (int32_t)mode_);
    TRACE_SET(rs, (int32_t)region_start_);
    TRACE_SET(rl, (int32_t)region_len_);
    TRACE_SET(pos, (float)pos_);
    TRACE_SET(step, step_);

    // Advance grain phase; restart grain 0 at 1.0 and grain 1 at 0.5 (50% overlap). A loop change
    // (SetRange) is taken at a grain start.
    const float prev = phase_;
    phase_ += phase_inc_;
    if (phase_ >= 1.0f) {
        phase_ -= 1.0f;
        StartGrain(0);
    }
    if (prev < 0.5f && phase_ >= 0.5f) StartGrain(1);

    TRACE_STAGE(kTrRead);
    float l, r;
    const double rel0 = Wrap(start_[0] + off_[0], glen_[0]);
    const double rel1 = Wrap(start_[1] + off_[1], glen_[1]);
    if (gbase_[0] == gbase_[1] && glen_[0] == glen_[1] && fabs(rel0 - rel1) < 1e-6) {
        // Both grains read the same point of the same loop (the usual case: they ride the
        // playhead) and their Hann windows add up to exactly 1, so one read is the whole output.
        ReadAt(rel0, (long)gbase_[0], (long)glen_[0], l, r);
    } else {
        // Grain 1's window is 1 - grain 0's (50 % overlap).
        float l0, r0, l1, r1;
        ReadAt(rel0, (long)gbase_[0], (long)glen_[0], l0, r0);
        ReadAt(rel1, (long)gbase_[1], (long)glen_[1], l1, r1);
        const float w0 = 0.5f - 0.5f * cosf(kTwoPi * phase_);
        l = l1 + w0 * (l0 - l1);
        r = r1 + w0 * (r0 - r1);
    }

    // Advance grain read offsets (integrated, so ratio changes don't make grains jump).
    off_[0] += gdir_[0] * ratio_;
    off_[1] += gdir_[1] * ratio_;

    out_l = l;
    out_r = r;
}
