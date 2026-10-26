#include "record_head.h"
#include <cmath>

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
}

void RecordHead::SetPreRoll(float* l, float* r, size_t len, size_t phase)
{
    pre_l_ = l;
    pre_r_ = r;
    pre_len_ = len;
    pre_phase_ = (len > 0) ? phase % len : 0;
}

void RecordHead::Process(double pos, float step, bool bounce, size_t region_start, size_t region_len,
                         long xfade, float in_l, float in_r, float decay)
{
    if (region_len < 2) return;
    // Delay the playhead, and the loop it is in, by kLag samples.
    if (hist_n_ < kLag) {
        hist_pos_[hist_n_] = pos;
        hist_step_[hist_n_] = step;
        hist_start_[hist_n_] = (long)region_start;
        hist_len_[hist_n_] = (long)region_len;
        hist_xf_[hist_n_] = xfade;
        hist_n_++;
        return;
    }
    const double p = hist_pos_[0];
    const float  s = hist_step_[0];
    const long   rs = hist_start_[0];
    const long   rl = hist_len_[0];
    const long   xf = hist_xf_[0];
    for (int i = 0; i < kLag - 1; i++) {
        hist_pos_[i] = hist_pos_[i + 1];
        hist_step_[i] = hist_step_[i + 1];
        hist_start_[i] = hist_start_[i + 1];
        hist_len_[i] = hist_len_[i + 1];
        hist_xf_[i] = hist_xf_[i + 1];
    }
    hist_pos_[kLag - 1] = pos;
    hist_step_[kLag - 1] = step;
    hist_start_[kLag - 1] = (long)region_start;
    hist_len_[kLag - 1] = (long)region_len;
    hist_xf_[kLag - 1] = xfade;

    if (!synced_) {
        synced_ = true;
        from_ = p;
        start_ = rs;
        len_ = rl;
        xf_ = xf;
        prev_l_ = in_l; prev_r_ = in_r;
        acc_l_ = acc_r_ = 0.0f; acc_n_ = 0;
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
            prev_l_ = in_l; prev_r_ = in_r;
            acc_l_ = acc_r_ = 0.0f; acc_n_ = 0;
            return;
        }
    }
    acc_l_ += in_l; acc_r_ += in_r; acc_n_++;

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
    for (long i = 0; i < count; i++) {
        const long k = first + i * dir;
        float x_l, x_r;
        if (count == 1) {
            // One sample passed: the average of the input since the last written sample.
            const float inv = 1.0f / (float)acc_n_;
            x_l = acc_l_ * inv;
            x_r = acc_r_ * inv;
        } else {
            // Several samples passed (fast): interpolate the input across the step.
            const float t = (float)(((double)k + 0.5 - from_) / (double)s);
            x_l = prev_l_ + t * (in_l - prev_l_);
            x_r = prev_r_ + t * (in_r - prev_r_);
        }
        if (changed && ((s > 0.0f) ? (k < edge) : (k >= edge)))
            Write(k - shift, bounce, (size_t)old_start, old_len, old_xf, x_l, x_r, decay);   // old loop
        else
            Write(k, bounce, (size_t)rs, rl, xf, x_l, x_r, decay);
    }
    if (count >= 1) { acc_l_ = acc_r_ = 0.0f; acc_n_ = 0; }
    prev_l_ = in_l; prev_r_ = in_r;
    from_ = p;
    xf_ = xf;
}

void RecordHead::Write(long k, bool bounce, size_t region_start, long region_len, long xf,
                       float x_l, float x_r, float decay)
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
    const size_t i = region_start + (size_t)k;
    buf_l_[i] = decay * buf_l_[i] + x_l;
    buf_r_[i] = decay * buf_r_[i] + x_r;
    // Over the crossfade, also write the lead sample the player blends in there (before the
    // loop start), so the new layer passes the crossfade at full level.
    if (xf < 1 || k < region_len - xf) return;
    const long m = (long)region_start - (region_len - k);
    if (m >= 0) {
        buf_l_[m] = decay * buf_l_[m] + x_l;
        buf_r_[m] = decay * buf_r_[m] + x_r;
    } else if (pre_len_ > 0 && -m <= (long)pre_len_) {
        const size_t j = (size_t)((long)pre_phase_ + (long)pre_len_ + m) % pre_len_;
        pre_l_[j] = decay * pre_l_[j] + x_l;
        pre_r_[j] = decay * pre_r_[j] + x_r;
    }
}
