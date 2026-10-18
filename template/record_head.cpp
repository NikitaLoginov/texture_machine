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

void RecordHead::Process(double pos, float step, bool bounce, size_t region_start, size_t region_len,
                         float in_l, float in_r, float decay)
{
    if (region_len < 2) return;
    // Delay the playhead by kLag samples.
    if (hist_n_ < kLag) {
        hist_pos_[hist_n_] = pos;
        hist_step_[hist_n_] = step;
        hist_n_++;
        return;
    }
    const double p = hist_pos_[0];
    const float  s = hist_step_[0];
    for (int i = 0; i < kLag - 1; i++) {
        hist_pos_[i] = hist_pos_[i + 1];
        hist_step_[i] = hist_step_[i + 1];
    }
    hist_pos_[kLag - 1] = pos;
    hist_step_[kLag - 1] = step;

    if (!synced_) {
        synced_ = true;
        from_ = p;
        prev_l_ = in_l; prev_r_ = in_r;
        acc_l_ = acc_r_ = 0.0f; acc_n_ = 0;
        return;
    }
    // Distance moved, across a region wrap. A jump (play restart, region moved): re-sync only.
    const double len = (double)region_len;
    double d = p - from_;
    if (d > 0.5 * len) d -= len;
    else if (d < -0.5 * len) d += len;
    if (d > 4.0 || d < -4.0) {
        from_ = p;
        prev_l_ = in_l; prev_r_ = in_r;
        acc_l_ = acc_r_ = 0.0f; acc_n_ = 0;
        return;
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
    if (count == 1) {
        // One sample passed: write the average of the input since the last written sample.
        const float inv = 1.0f / (float)acc_n_;
        Write(first, bounce, region_start, (long)region_len, acc_l_ * inv, acc_r_ * inv, decay);
    } else if (count > 1) {
        // Several samples passed (fast): interpolate the input across the step.
        for (long i = 0; i < count; i++) {
            const long k = first + i * dir;
            const float t = (float)(((double)k + 0.5 - from_) / (double)s);
            Write(k, bounce, region_start, (long)region_len,
                  prev_l_ + t * (in_l - prev_l_), prev_r_ + t * (in_r - prev_r_), decay);
        }
    }
    if (count >= 1) { acc_l_ = acc_r_ = 0.0f; acc_n_ = 0; }
    prev_l_ = in_l; prev_r_ = in_r;
    from_ = p;
}

void RecordHead::Write(long k, bool bounce, size_t region_start, long region_len,
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
}
