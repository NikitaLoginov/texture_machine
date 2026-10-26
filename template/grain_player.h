#pragma once
#include <cstddef>

class GrainPlayer {
public:
    enum Mode { FORWARD = 0, PINGPONG = 1, BACKWARD = 2 };

    void Init(const float* buf_l, const float* buf_r, size_t capacity, float sample_rate);
    void SetLength(size_t length);     // valid recorded samples, clamp to 0..capacity
    void SetRange(float a, float b);   // loop start a (0..1 of length), b = loop length as 0..1 of the rest after the start; applied through the loop crossfade at the next wrap (see Process), or on Reset()
    void SetMode(Mode mode);
    void SetSpeed(float speed);        // playhead speed, 1 = normal, 0 = freeze, clamp to >= 0
    void SetPitch(float semitones);    // grain transposition, clamp -36..36
    void SetGrainMs(float ms);         // grain length in ms, clamp 5..1000
    void SetFadeMs(float ms);          // loop crossfade length, clamp 1..2000

    // Lead-in: the input just before the recording, a ring of len samples per channel whose
    // oldest sample is at index phase. Lead-in sample j (-len .. -1, -1 = just before the
    // recording) is at ring index (phase + len + j) % len.
    void SetPreRoll(const float* l, const float* r, size_t len, size_t phase);

    // Overdub track: same positions as the recording, read with its own short seam at the loop
    // end (kOvdXfade), not the S33 crossfade. Lead-in before sample 0: sample m < 0 at index len + m.
    void SetOverdub(const float* l, const float* r, const float* pre_l, const float* pre_r, size_t pre_len);
    void Reset();                      // playhead and grains back to region start
    void Process(float& out_l, float& out_r);  // one stereo frame

    // For the overdub record head:
    double Pos() const { return pos_; }                      // playhead inside the region, samples
    float  Step() const { return step_; }                    // signed move in the last Process(), before wrap/bounce
    bool   Bounces() const { return mode_ == PINGPONG; }
    size_t RegionStart() const { return (size_t)region_start_; }
    size_t RegionLen() const { return (size_t)region_len_; }
    long   RegionXfade() const { return Xfade((long)region_len_); }   // crossfade of the current loop, samples
    size_t RegionLead() const { return (size_t)lead_start_; }        // the crossfade lead is the audio before this sample
    long   RegionOvdXfade() const { return OvdXfade((long)region_len_); }   // overdub track seam, samples

private:
    void UpdateRegion();                                // apply the loop set by SetRange now
    void NextRegion(double& start, double& len) const;  // the loop set by SetRange
    static double Wrap(double x, double len);
    long Xfade(long n) const;
    void Lead(long m, float& out_l, float& out_r) const;
    void LoopSample(long n, long xf, long xo, long k, float& out_l, float& out_r) const;
    long OvdXfade(long n) const;
    void OvdLead(long m, float& out_l, float& out_r) const;
    static constexpr long kOvdXfade = 480;   // overdub track seam at the loop end, samples (10 ms)
    static constexpr double kLeadMargin = 960.0;   // a loop change is fixed this long before the crossfade (samples)

    const float* buf_l_;
    const float* buf_r_;
    const float* pre_l_;
    const float* pre_r_;
    size_t pre_len_;     // lead-in length, samples (0 = none)
    size_t pre_phase_;   // ring index of the oldest lead-in sample
    const float* ovd_l_;      // overdub track (nullptr = none)
    const float* ovd_r_;
    const float* ovd_pre_l_;  // its lead-in before sample 0
    const float* ovd_pre_r_;
    size_t ovd_pre_len_;
    size_t capacity_;
    float sr_;

    size_t length_;
    float range_a_;
    float range_b_;
    double region_start_;
    double region_len_;
    double lead_start_;   // the crossfade lead is the audio before this sample (normally the loop start)
    bool   latched_;      // forward: a loop change is fixed for the coming wrap
    double next_start_;   // that loop
    double next_len_;

    Mode mode_;
    float speed_;
    float ratio_;
    float grain_len_;
    float phase_inc_;
    float fade_len_;   // samples

    double pos_;
    float dir_;
    float step_;

    float phase_;
    double start_[2];
    float gdir_[2];
    double off_[2];     // per-grain read offset from start_, advanced by gdir_ * ratio_
};
