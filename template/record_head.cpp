#include "record_head.h"
#include <cmath>

// Soft limit on what is written: unchanged below kKnee, then bends smoothly toward 1.0.
static constexpr float kKnee = 0.5f;

static float Sat(float v)
{
    const float a = fabsf(v);
    if (a <= kKnee) return v;
    const float y = kKnee + (1.0f - kKnee) * tanhf((a - kKnee) / (1.0f - kKnee));
    return (v < 0.0f) ? -y : y;
}

// One overdub write: crossfade (by gain) from the old value to the limited new layer sum.
// aged: the old value after one generation of tape loss.
static float Dub(float old, float aged, float x, float gain, float keep)
{
    return old + gain * (Sat(keep * aged + x) - old);
}

void RecordHead::Init(float* buf_l, float* buf_r)
{
    buf_l_ = buf_l;
    buf_r_ = buf_r;
    Start();
}

void RecordHead::Start()
{
    synced_ = false;
    hist_n_ = 0;
    lp_init_ = false;
    age_init_ = mage_init_ = false;
    wob_len_ = wob_left_ = wob_gap_ = 0;
    last_k_ = -1;
}

void RecordHead::SetPreRoll(float* l, float* r, size_t len, size_t phase)
{
    pre_l_ = l;
    pre_r_ = r;
    pre_len_ = len;
    pre_phase_ = (len > 0) ? phase % len : 0;
}

bool RecordHead::Pending() const
{
    for (int i = 0; i < hist_n_; i++)
        if (hist_gain_[i] > 0.0f) return true;
    return false;
}

float RecordHead::Noise()
{
    seed_ ^= seed_ << 13;
    seed_ ^= seed_ >> 17;
    seed_ ^= seed_ << 5;
    return (float)(int32_t)seed_ * (1.0f / 2147483648.0f);
}

// Printed tape stutter on the old layers: steady tape, then at random (wob_rate_ chance per
// written sample) a burst of kBurstMin..kBurstMax jerks. Each jerk reads the old content ahead
// of the head by a short sin^2 bump (pitch quickly up and back, peak up to wob_pitch_), followed
// by a short stretch of steady tape. A jerk only starts if it stays inside the loop, away from
// the edges and the crossfade (the read stays on not-yet-written samples); else the burst ends.
double RecordHead::Wobble(long k, long region_len, long xf)
{
    if (wob_len_ == 0) {
        if (wob_left_ > 0) {
            if (wob_gap_ > 0) { wob_gap_--; return 0.0; }
        } else {
            if (0.5f + 0.5f * Noise() >= wob_rate_) return 0.0;
            wob_left_ = kBurstMin + (long)((0.5f + 0.5f * Noise()) * (float)(kBurstMax - kBurstMin + 1));
        }
        const long len = kWobMin + (long)((0.5f + 0.5f * Noise()) * (float)(kWobMax - kWobMin));
        const double reach = (double)(wob_pitch_ * (0.5f + 0.5f * Noise())) * (double)len / 3.14159265;
        const long lo = (long)kEdgeFade;
        const long hi = region_len - xf - (long)kEdgeFade;
        const long end = k + dir_ * (len + (long)reach + 2);
        if (k < lo || k >= hi || end < lo || end >= hi) { wob_left_ = 0; return 0.0; }
        wob_len_ = len;
        wob_pos_ = 0;
        wob_reach_ = reach;
        wob_left_--;
        wob_gap_ = (long)((0.5f + 0.5f * Noise()) * (float)kWobGapMax);
    }
    const double w = sin(3.14159265 * (double)wob_pos_ / (double)wob_len_);
    if (++wob_pos_ >= wob_len_) wob_len_ = 0;
    return (double)dir_ * wob_reach_ * w * w;
}

void RecordHead::Process(double pos, float step, bool bounce, size_t region_start, size_t region_len,
                         long xfade, float in_l, float in_r, float gain, float keep)
{
    if (region_len < 2) return;
    // Delay the playhead, the loop it is in and the write gain by kLag samples.
    if (hist_n_ < kLag) {
        hist_pos_[hist_n_] = pos;
        hist_step_[hist_n_] = step;
        hist_start_[hist_n_] = (long)region_start;
        hist_len_[hist_n_] = (long)region_len;
        hist_xf_[hist_n_] = xfade;
        hist_gain_[hist_n_] = gain;
        hist_n_++;
        return;
    }
    const double p = hist_pos_[0];
    const float  s = hist_step_[0];
    const long   rs = hist_start_[0];
    const long   rl = hist_len_[0];
    const long   xf = hist_xf_[0];
    const float  g = hist_gain_[0];
    for (int i = 0; i < kLag - 1; i++) {
        hist_pos_[i] = hist_pos_[i + 1];
        hist_step_[i] = hist_step_[i + 1];
        hist_start_[i] = hist_start_[i + 1];
        hist_len_[i] = hist_len_[i + 1];
        hist_xf_[i] = hist_xf_[i + 1];
        hist_gain_[i] = hist_gain_[i + 1];
    }
    hist_pos_[kLag - 1] = pos;
    hist_step_[kLag - 1] = step;
    hist_start_[kLag - 1] = (long)region_start;
    hist_len_[kLag - 1] = (long)region_len;
    hist_xf_[kLag - 1] = xfade;
    hist_gain_[kLag - 1] = gain;

    // Input low-pass, coefficient = tape speed (anti-aliasing when the tape runs slow).
    const float a = (fabsf(s) < 1.0f) ? fabsf(s) : 1.0f;
    if (!lp_init_) { lp_l_ = in_l; lp_r_ = in_r; lp_init_ = true; }
    lp_l_ += a * (in_l - lp_l_);
    lp_r_ += a * (in_r - lp_r_);
    const float x_now_l = lp_l_, x_now_r = lp_r_;

    if (!synced_) {
        synced_ = true;
        from_ = p;
        start_ = rs;
        len_ = rl;
        xf_ = xf;
        prev_l_ = x_now_l; prev_r_ = x_now_r;
        return;
    }
    // The player changes the loop only at a wrap or bounce. Then move the head across that edge
    // into the new loop; samples still on the old side of the edge are written to the old loop.
    const long old_start = start_, old_len = len_, old_xf = xf_;
    bool changed = false;
    long edge = 0, shift = 0;
    if (rs != start_ || rl != len_) {
        changed = true;
        const long old_edge = (s > 0.0f) ? old_len : 0;
        if (s > 0.0f) edge = bounce ? rl : 0;
        else          edge = bounce ? 0 : rl;
        shift = edge - old_edge;
        from_ += (double)shift;
        start_ = rs;
        len_ = rl;
    } else {
        // Distance moved, across a region wrap. A jump (play restart): re-sync only.
        const double len = (double)rl;
        double d = p - from_;
        if (d > 0.5 * len) d -= len;
        else if (d < -0.5 * len) d += len;
        if (d > 4.0 || d < -4.0) {
            from_ = p;
            prev_l_ = x_now_l; prev_r_ = x_now_r;
            return;
        }
    }

    // Buffer sample k is passed when the head crosses k + 0.5 on its way from from_ to from_ + s.
    const double to = from_ + s;
    long first = 0, count = 0, dir = 1;
    if (s > 0.0f) {
        first = (long)floor(from_ - 0.5) + 1;
        count = (long)floor(to - 0.5) - first + 1;
    } else if (s < 0.0f) {
        dir = -1;
        first = (long)ceil(from_ - 0.5) - 1;
        count = first - (long)ceil(to - 0.5) + 1;
    }
    dir_ = (dir > 0) ? 1 : -1;
    for (long i = 0; i < count; i++) {
        const long k = first + i * dir;
        // Input at the moment the head crosses k + 0.5 (interpolated across this step).
        const float t = (float)(((double)k + 0.5 - from_) / (double)s);
        const float x_l = prev_l_ + t * (x_now_l - prev_l_);
        const float x_r = prev_r_ + t * (x_now_r - prev_r_);
        if (changed && ((s > 0.0f) ? (k < edge) : (k >= edge)))
            Write(k - shift, bounce, (size_t)old_start, old_len, old_xf, x_l, x_r, g, keep);   // old loop
        else
            Write(k, bounce, (size_t)rs, rl, xf, x_l, x_r, g, keep);
    }
    prev_l_ = x_now_l; prev_r_ = x_now_r;
    from_ = p;
    xf_ = xf;
}

void RecordHead::Write(long k, bool bounce, size_t region_start, long region_len, long xf,
                       float x_l, float x_r, float gain, float keep)
{
    // Fold k into the region: mirror at the edges in ping-pong, wrap otherwise.
    if (bounce) {
        if (k < 0) k = -k - 1;
        else if (k >= region_len) k = 2 * region_len - k - 1;
    } else {
        k %= region_len;
        if (k < 0) k += region_len;
    }
    if (k < 0 || k >= region_len) return;
    // Ping-pong: fade the write near the edges, where the playhead turns back into the head.
    if (bounce) {
        const long dist = (k < region_len - 1 - k) ? k : region_len - 1 - k;
        const float e = ((float)dist + 0.5f) / kEdgeFade;
        if (e < 1.0f) gain *= e;
    }
    // At the loop wrap the aging low-pass continues from what is heard just before it: forward
    // (end -> start) the start continues from the crossfade lead, backward (start -> end) the
    // lead continues from the start. Without this the filter prints a step at the wrap.
    if (dir_ > 0 && k < last_k_ - 1 && mage_init_) {
        age_l_ = mage_l_; age_r_ = mage_r_; age_init_ = true;
    } else if (dir_ < 0 && k > last_k_ + 1 && age_init_) {
        mage_l_ = age_l_; mage_r_ = age_r_; mage_init_ = true;
    }
    last_k_ = k;
    const size_t i = region_start + (size_t)k;
    // Old layers as read with the printed wobble: at k, or a little ahead of it (interpolated).
    const double rd = (double)k + Wobble(k, region_len, xf);
    const size_t j0 = region_start + (size_t)rd;
    const float fr = (float)(rd - floor(rd));
    const float old_l = (fr > 0.0f) ? buf_l_[j0] + fr * (buf_l_[j0 + 1] - buf_l_[j0]) : buf_l_[j0];
    const float old_r = (fr > 0.0f) ? buf_r_[j0] + fr * (buf_r_[j0 + 1] - buf_r_[j0]) : buf_r_[j0];
    // Age the old layers by one tape generation: hiss, then the low-pass (in write order).
    if (!age_init_) { age_l_ = old_l; age_r_ = old_r; age_init_ = true; }
    age_l_ += age_coef_ * (old_l + age_hiss_ * Noise() - age_l_);
    age_r_ += age_coef_ * (old_r + age_hiss_ * Noise() - age_r_);
    buf_l_[i] = Dub(buf_l_[i], age_l_, x_l, gain, keep);
    buf_r_[i] = Dub(buf_r_[i], age_r_, x_r, gain, keep);
    // Over the crossfade, also write the lead sample the player blends in there (before the
    // loop start), so the new layer passes the crossfade at full level. The lead samples are
    // a separate sequence with their own aging state, restarted when the head enters the crossfade.
    if (xf < 1 || k < region_len - xf) { mage_init_ = false; return; }
    const long m = (long)region_start - (region_len - k);
    float* l = nullptr;
    float* r = nullptr;
    if (m >= 0) {
        l = &buf_l_[m];
        r = &buf_r_[m];
    } else if (pre_len_ > 0 && -m <= (long)pre_len_) {
        const size_t j = (size_t)((long)pre_phase_ + (long)pre_len_ + m) % pre_len_;
        l = &pre_l_[j];
        r = &pre_r_[j];
    }
    if (l == nullptr) return;
    if (!mage_init_) { mage_l_ = *l; mage_r_ = *r; mage_init_ = true; }
    mage_l_ += age_coef_ * (*l + age_hiss_ * Noise() - mage_l_);
    mage_r_ += age_coef_ * (*r + age_hiss_ * Noise() - mage_r_);
    *l = Dub(*l, mage_l_, x_l, gain, keep);
    *r = Dub(*r, mage_r_, x_r, gain, keep);
}
