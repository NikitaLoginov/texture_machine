#pragma once
#include <cstddef>

// Tape-style overdub record head. Follows the GrainPlayer playhead, 2 samples behind it, and
// writes new = decay * old + input into every buffer sample it passes, once per pass.
class RecordHead {
public:
    void Init(float* buf_l, float* buf_r);
    void Start();   // overdub starts: re-sync to the playhead, write nothing yet
    // Lead-in ring (same as GrainPlayer::SetPreRoll): len samples, oldest at index phase.
    void SetPreRoll(float* l, float* r, size_t len, size_t phase);
    // Once per sample while overdubbing, after GrainPlayer::Process().
    // pos: playhead position inside the loop region (0 .. region_len).
    // step: signed distance the playhead moved this sample, before wrap or bounce.
    // bounce: true in ping-pong mode (the playhead bounces at the region edges instead of wrapping).
    // region_start, region_len: loop region in whole samples. xfade: GrainPlayer::RegionXfade().
    void Process(double pos, float step, bool bounce, size_t region_start, size_t region_len, long xfade,
                 float in_l, float in_r, float decay);

private:
    void Write(long k, bool bounce, size_t region_start, long region_len, long xf,
               float x_l, float x_r, float decay);
    static constexpr int kLag = 2;              // samples behind the playhead
    float* buf_l_ = nullptr;
    float* buf_r_ = nullptr;
    double hist_pos_[kLag] = {};                // delayed playhead positions
    float  hist_step_[kLag] = {};               // delayed playhead steps
    long   hist_start_[kLag] = {};              // loop start of each delayed position
    long   hist_len_[kLag] = {};                // loop length of each delayed position
    long   hist_xf_[kLag] = {};                 // crossfade length of each delayed position
    int    hist_n_ = 0;                         // filled history entries, 0 .. kLag
    bool   synced_ = false;
    double from_ = 0.0;                         // head position after the last step
    long   start_ = 0, len_ = 0;                // loop the head was in at from_
    long   xf_ = 0;                             // crossfade length of that loop
    float* pre_l_ = nullptr;                    // lead-in ring (before the recording)
    float* pre_r_ = nullptr;
    size_t pre_len_ = 0, pre_phase_ = 0;
    float  prev_l_ = 0.0f, prev_r_ = 0.0f;      // input at the last step
    float  acc_l_ = 0.0f, acc_r_ = 0.0f;        // input summed since the last written sample
    int    acc_n_ = 0;
};
