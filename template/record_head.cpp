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

void RecordHead::Init(float* buf_l, float* buf_r, float* ovd_l, float* ovd_r, size_t capacity)
{
    trk_[0].l = buf_l;
    trk_[0].r = buf_r;
    trk_[1].l = ovd_l;
    trk_[1].r = ovd_r;
    cap_ = capacity;
    for (long j = 0; j < kTail; j++) {
        const float c = cosf(1.5707963f * (float)j / (float)kTail);
        tail_w_[j] = c * c;
    }
    Start();
}

void RecordHead::Start()
{
    synced_ = false;
    hist_n_ = 0;
    lp_init_ = false;
    for (int n = 0; n < 2; n++) trk_[n].age_init = trk_[n].mage_init = false;
    wob_len_ = wob_left_ = wob_gap_ = 0;
    last_k_ = -1;
}

void RecordHead::SetPreRoll(float* l, float* r, size_t len, size_t phase)
{
    trk_[0].pre_l = l;
    trk_[0].pre_r = r;
    trk_[0].pre_len = len;
    trk_[0].pre_phase = (len > 0) ? phase % len : 0;
}

void RecordHead::SetOverdubPreRoll(float* l, float* r, size_t len)
{
    trk_[1].pre_l = l;
    trk_[1].pre_r = r;
    trk_[1].pre_len = len;
    trk_[1].pre_phase = 0;
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

// Pointers to sample m of a track: its buffer for m >= 0, its lead-in ring before sample 0.
bool RecordHead::Slot(Track& t, long m, float*& pl, float*& pr)
{
    if (m >= 0) {
        if (m >= (long)cap_) return false;
        pl = &t.l[m];
        pr = &t.r[m];
        return true;
    }
    if (t.pre_len == 0 || -m > (long)t.pre_len) return false;
    const size_t j = (size_t)((long)t.pre_phase + (long)t.pre_len + m) % t.pre_len;
    pl = &t.pre_l[j];
    pr = &t.pre_r[j];
    return true;
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
                         long xfade, long ovd_xfade, size_t lead, float in_l, float in_r, float gain, float keep)
{
    if (region_len < 2) return;
    // Delay the playhead, the loop it is in and the write gain by kLag samples.
    if (hist_n_ < kLag) {
        hist_pos_[hist_n_] = pos;
        hist_step_[hist_n_] = step;
        hist_start_[hist_n_] = (long)region_start;
        hist_len_[hist_n_] = (long)region_len;
        hist_xf_[hist_n_] = xfade;
        hist_xo_[hist_n_] = ovd_xfade;
        hist_lead_[hist_n_] = (long)lead;
        hist_gain_[hist_n_] = gain;
        hist_n_++;
        return;
    }
    const double p = hist_pos_[0];
    const float  s = hist_step_[0];
    const long   rs = hist_start_[0];
    const long   rl = hist_len_[0];
    const long   xf = hist_xf_[0];
    const long   xo = hist_xo_[0];
    const long   ld = hist_lead_[0];
    const float  g = hist_gain_[0];
    for (int i = 0; i < kLag - 1; i++) {
        hist_pos_[i] = hist_pos_[i + 1];
        hist_step_[i] = hist_step_[i + 1];
        hist_start_[i] = hist_start_[i + 1];
        hist_len_[i] = hist_len_[i + 1];
        hist_xf_[i] = hist_xf_[i + 1];
        hist_xo_[i] = hist_xo_[i + 1];
        hist_lead_[i] = hist_lead_[i + 1];
        hist_gain_[i] = hist_gain_[i + 1];
    }
    hist_pos_[kLag - 1] = pos;
    hist_step_[kLag - 1] = step;
    hist_start_[kLag - 1] = (long)region_start;
    hist_len_[kLag - 1] = (long)region_len;
    hist_xf_[kLag - 1] = xfade;
    hist_xo_[kLag - 1] = ovd_xfade;
    hist_lead_[kLag - 1] = (long)lead;
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
        xo_ = xo;
        lead_ = ld;
        prev_l_ = x_now_l; prev_r_ = x_now_r;
        return;
    }
    // The player changes the loop only at a wrap or bounce. Then move the head across that edge
    // into the new loop; samples still on the old side of the edge are written to the old loop.
    const long old_start = start_, old_len = len_, old_xf = xf_, old_xo = xo_, old_lead = lead_;
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
            Write(k - shift, bounce, (size_t)old_start, old_len, old_xf, old_xo, old_lead, x_l, x_r, g, keep);   // old loop
        else
            Write(k, bounce, (size_t)rs, rl, xf, xo, ld, x_l, x_r, g, keep);
    }
    prev_l_ = x_now_l; prev_r_ = x_now_r;
    from_ = p;
    xf_ = xf;
    xo_ = xo;
    lead_ = ld;
}

void RecordHead::Write(long k, bool bounce, size_t region_start, long region_len, long xf, long xo,
                       long lead, float x_l, float x_r, float gain, float keep)
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
    // At the loop wrap each track's aging low-pass continues from what is heard just before it:
    // forward (end -> start) the start continues from the lead, backward (start -> end) the
    // lead continues from the start. Without this the filter prints a step at the wrap.
    for (int n = 0; n < 2; n++) {
        Track& t = trk_[n];
        if (dir_ > 0 && k < last_k_ - 1 && t.mage_init) {
            t.age_l = t.mage_l; t.age_r = t.mage_r; t.age_init = true;
        } else if (dir_ < 0 && k > last_k_ + 1 && t.age_init) {
            t.mage_l = t.age_l; t.mage_r = t.age_r; t.mage_init = true;
        }
    }
    last_k_ = k;
    const size_t i = region_start + (size_t)k;
    // Old layers of both tracks as read with the printed stutter: at k, or a little ahead of it.
    const double rd = (double)k + Wobble(k, region_len, xf);
    const size_t j0 = region_start + (size_t)rd;
    const float fr = (float)(rd - floor(rd));
    float old_l[2], old_r[2];
    for (int n = 0; n < 2; n++) {
        const Track& t = trk_[n];
        old_l[n] = (fr > 0.0f) ? t.l[j0] + fr * (t.l[j0 + 1] - t.l[j0]) : t.l[j0];
        old_r[n] = (fr > 0.0f) ? t.r[j0] + fr * (t.r[j0 + 1] - t.r[j0]) : t.r[j0];
    }
    // Generation noise, baked into the material: it follows the level of the old layers
    // (5 ms up, 50 ms down) over a quiet floor, and fades in over the first kGenRamp passes.
    const float lvl = 0.5f * (fabsf(old_l[0] + old_l[1]) + fabsf(old_r[0] + old_r[1]));
    env_ += ((lvl > env_) ? 0.004f : 0.0004f) * (lvl - env_);
    if (gen_ < kGenRamp) gen_ += 1.0f / (float)region_len;
    const float ramp = (gen_ < kGenRamp) ? gen_ / kGenRamp : 1.0f;
    const float hiss = age_hiss_ * ramp * (kNoiseFloor + kNoiseMod * env_);
    // The overdub track's own seam at the loop end: the input goes cos into the loop end and
    // sin into the lead before the start (the player blends them equal-power).
    float in_end = 1.0f, in_lead = 0.0f;
    if (xo >= 1 && k >= region_len - xo) {
        const float u = 1.5707963f * (float)(k - (region_len - xo)) / (float)xo;
        in_end = cosf(u);
        in_lead = sinf(u);
    }
    for (int n = 0; n < 2; n++) {
        Track& t = trk_[n];
        const float h = (n == 0) ? hiss : 0.0f;      // generation noise: on the recording track
        const float xi = (n == 0) ? 0.0f : in_end;   // the input: on the overdub track only
        // Age the old layers by one tape generation: noise, then the low-pass (in write order).
        if (!t.age_init) { t.age_l = old_l[n]; t.age_r = old_r[n]; t.age_init = true; }
        t.age_l += age_coef_ * (old_l[n] + h * Noise() - t.age_l);
        t.age_r += age_coef_ * (old_r[n] + h * Noise() - t.age_r);
        t.l[i] = Dub(t.l[i], t.age_l, xi * x_l, gain, keep);
        t.r[i] = Dub(t.r[i], t.age_r, xi * x_r, gain, keep);
        // Past the loop end (not in ping-pong): the decay fades out over kTail samples, so the
        // level of the old layers has no step there if the loop later ends later.
        if (!bounce && k < kTail) {
            const long e = (long)region_start + region_len + k;
            if (e < (long)cap_) {
                const float g = gain * tail_w_[k];
                t.l[e] = Dub(t.l[e], t.l[e], 0.0f, g, keep);
                t.r[e] = Dub(t.r[e], t.r[e], 0.0f, g, keep);
            }
        }
        // Lead zone before the loop start, which the player blends in at the loop end: the
        // recording over the S33 crossfade xf, the overdub track over its seam xo (with the sin
        // part of the input). kTail samples more before it fade the write in (sin^2).
        const long xz = (n == 0) ? xf : xo;
        if (xz < 1 || k < region_len - xz - kTail) { t.mage_init = false; continue; }
        float mg = gain;
        float xm = 0.0f;
        if (k < region_len - xz) {
            const float sw = sinf(1.5707963f * (float)(k - (region_len - xz - kTail)) / (float)kTail);
            mg *= sw * sw;
        } else if (n == 1) {
            xm = in_lead;
        }
        float* pl;
        float* pr;
        if (!Slot(t, lead - (region_len - k), pl, pr)) continue;
        if (!t.mage_init) { t.mage_l = *pl; t.mage_r = *pr; t.mage_init = true; }
        t.mage_l += age_coef_ * (*pl + h * Noise() - t.mage_l);
        t.mage_r += age_coef_ * (*pr + h * Noise() - t.mage_r);
        *pl = Dub(*pl, t.mage_l, xm * x_l, mg, keep);
        *pr = Dub(*pr, t.mage_r, xm * x_r, mg, keep);
    }
}
