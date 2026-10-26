#pragma once
#include <cstddef>
#include <cstdint>

// Tape-style overdub record head. Follows the GrainPlayer playhead, 8 samples behind it, and
// writes new = old + gain * (Sat(keep * old + input) - old) into every buffer sample it passes, once per pass.
class RecordHead {
public:
    void Init(float* buf_l, float* buf_r);
    void Start();   // overdub starts: re-sync to the playhead, write nothing yet
    // Lead-in ring (same as GrainPlayer::SetPreRoll): len samples, oldest at index phase.
    void SetPreRoll(float* l, float* r, size_t len, size_t phase);
    bool Pending() const;   // delayed writes still to do: keep calling Process() while true
    // Aging of the old layers per pass: one-pole low-pass coefficient (1 = no loss), hiss level,
    // stutter burst chance per written sample and the peak pitch deviation of its jerks (0.01 = 1 %).
    void SetAging(float lp_coef, float hiss, float wob_rate, float wob_pitch)
    {
        age_coef_ = lp_coef;
        age_hiss_ = hiss;
        wob_rate_ = wob_rate;
        wob_pitch_ = wob_pitch;
    }
    // Once per sample while overdubbing, after GrainPlayer::Process().
    // pos: playhead position inside the loop region (0 .. region_len).
    // step: signed distance the playhead moved this sample, before wrap or bounce.
    // bounce: true in ping-pong mode (the playhead bounces at the region edges instead of wrapping).
    // region_start, region_len: loop region in whole samples. xfade: GrainPlayer::RegionXfade().
    // gain: write level 0..1 (0 = buffer untouched). keep: share of the old layers kept per pass.
    void Process(double pos, float step, bool bounce, size_t region_start, size_t region_len, long xfade,
                 float in_l, float in_r, float gain, float keep);

private:
    void Write(long k, bool bounce, size_t region_start, long region_len, long xf,
               float x_l, float x_r, float gain, float keep);
    float Noise();                              // white noise -1 .. 1
    double Wobble(long k, long region_len, long xf);   // printed-wobble read offset for loop sample k
    static constexpr int kLag = 8;              // samples behind the playhead
    static constexpr float kEdgeFade = 480.0f;  // ping-pong: write fade near the edges, samples
    static constexpr long kWobMin = 1440;       // stutter jerk length 30 .. 150 ms, buffer samples
    static constexpr long kWobMax = 7200;
    static constexpr long kWobGapMax = 2880;    // steady tape after each jerk of a burst, 0 .. 60 ms
    static constexpr long kBurstMin = 2;        // jerks per burst
    static constexpr long kBurstMax = 6;
    float* buf_l_ = nullptr;
    float* buf_r_ = nullptr;
    double hist_pos_[kLag] = {};                // delayed playhead positions
    float  hist_step_[kLag] = {};               // delayed playhead steps
    long   hist_start_[kLag] = {};              // loop start of each delayed position
    long   hist_len_[kLag] = {};                // loop length of each delayed position
    long   hist_xf_[kLag] = {};                 // crossfade length of each delayed position
    float  hist_gain_[kLag] = {};               // write gain of each delayed position
    int    hist_n_ = 0;                         // filled history entries, 0 .. kLag
    bool   synced_ = false;
    double from_ = 0.0;                         // head position after the last step
    long   start_ = 0, len_ = 0;                // loop the head was in at from_
    long   xf_ = 0;                             // crossfade length of that loop
    float* pre_l_ = nullptr;                    // lead-in ring (before the recording)
    float* pre_r_ = nullptr;
    size_t pre_len_ = 0, pre_phase_ = 0;
    float  prev_l_ = 0.0f, prev_r_ = 0.0f;      // input at the last step
    float  lp_l_ = 0.0f, lp_r_ = 0.0f;          // input low-pass (follows the tape speed)
    bool   lp_init_ = false;
    float  age_coef_ = 1.0f, age_hiss_ = 0.0f;  // aging set by SetAging()
    float  age_l_ = 0.0f, age_r_ = 0.0f;        // aging low-pass state, loop samples
    float  mage_l_ = 0.0f, mage_r_ = 0.0f;      // aging low-pass state, crossfade lead samples
    bool   age_init_ = false, mage_init_ = false;
    uint32_t seed_ = 0x1234567u;                // noise state
    float  wob_rate_ = 0.0f, wob_pitch_ = 0.0f;     // wobble event chance per sample, peak pitch deviation
    long   wob_len_ = 0, wob_pos_ = 0;              // current wobble event length (0 = none) and position
    double wob_reach_ = 0.0;                        // its peak read-ahead, samples
    int    dir_ = 1;                                // head direction in the buffer, +1 / -1
    long   wob_left_ = 0, wob_gap_ = 0;             // jerks left in the burst, steady samples before the next
    long   last_k_ = -1;                            // loop sample written last (to spot the wrap)
};
