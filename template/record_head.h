#pragma once
#include <cstddef>

// Tape-style overdub record head. Follows the GrainPlayer playhead, 2 samples behind it, and
// writes new = decay * old + input into every buffer sample it passes, once per pass.
class RecordHead {
public:
    void Init(float* buf_l, float* buf_r);
    void Start();   // overdub starts: re-sync to the playhead, write nothing yet
    // Once per sample while overdubbing, after GrainPlayer::Process().
    // pos: playhead position inside the loop region (0 .. region_len).
    // step: signed distance the playhead moved this sample, before wrap or bounce.
    // bounce: true in ping-pong mode (the playhead bounces at the region edges instead of wrapping).
    // region_start, region_len: loop region in whole samples. decay: share of old content kept, 0..1.
    void Process(double pos, float step, bool bounce, size_t region_start, size_t region_len,
                 float in_l, float in_r, float decay);

private:
    void Write(long k, bool bounce, size_t region_start, long region_len,
               float x_l, float x_r, float decay);
    static constexpr int kLag = 2;              // samples behind the playhead
    float* buf_l_ = nullptr;
    float* buf_r_ = nullptr;
    double hist_pos_[kLag] = {};                // delayed playhead positions
    float  hist_step_[kLag] = {};               // delayed playhead steps
    int    hist_n_ = 0;                         // filled history entries, 0 .. kLag
    bool   synced_ = false;
    double from_ = 0.0;                         // head position after the last step
    float  prev_l_ = 0.0f, prev_r_ = 0.0f;      // input at the last step
    float  acc_l_ = 0.0f, acc_r_ = 0.0f;        // input summed since the last written sample
    int    acc_n_ = 0;
};
