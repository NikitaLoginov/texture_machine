#pragma once
#include <cstddef>

class GrainPlayer {
public:
    enum Mode { FORWARD = 0, PINGPONG = 1, BACKWARD = 2 };

    void Init(const float* buf_l, const float* buf_r, size_t capacity, float sample_rate);
    void SetLength(size_t length);     // valid recorded samples, clamp to 0..capacity
    void SetRange(float a, float b);   // loop start a (0..1 of length), b = loop length as 0..1 of the rest after the start; applied at the end of the current pass, or on Reset()
    void SetMode(Mode mode);
    void SetSpeed(float speed);        // playhead speed, 1 = normal, 0 = freeze, clamp to >= 0
    void SetPitch(float semitones);    // grain transposition, clamp -24..24
    void SetGrainMs(float ms);         // grain length in ms, clamp 5..1000
    void SetFadeMs(float ms);          // loop crossfade length, clamp 1..2000

    // Lead-in: the input just before the recording, a ring of len samples per channel whose
    // oldest sample is at index phase. Lead-in sample j (-len .. -1, -1 = just before the
    // recording) is at ring index (phase + len + j) % len.
    void SetPreRoll(const float* l, const float* r, size_t len, size_t phase);
    void Reset();                      // playhead and grains back to region start
    void Process(float& out_l, float& out_r);  // one stereo frame

    // For the overdub record head:
    double Pos() const { return pos_; }                      // playhead inside the region, samples
    float  Step() const { return step_; }                    // signed move in the last Process(), before wrap/bounce
    bool   Bounces() const { return mode_ == PINGPONG; }
    size_t RegionStart() const { return (size_t)region_start_; }
    size_t RegionLen() const { return (size_t)region_len_; }
    long   RegionXfade() const { return Xfade((long)region_start_, (long)region_len_); }   // crossfade of the current loop, samples

private:
    void UpdateRegion();
    static double Wrap(double x, double len);
    long Xfade(long base, long n) const;
    void LoopSample(long base, long n, long xf, long k, float& out_l, float& out_r) const;

    const float* buf_l_;
    const float* buf_r_;
    const float* pre_l_;
    const float* pre_r_;
    size_t pre_len_;     // lead-in length, samples (0 = none)
    size_t pre_phase_;   // ring index of the oldest lead-in sample
    size_t capacity_;
    float sr_;

    size_t length_;
    float range_a_;
    float range_b_;
    double region_start_;
    double region_len_;

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
    double gbase_[2];   // per-grain region start (samples), taken when the grain starts
    double glen_[2];    // per-grain region length (samples), taken when the grain starts
};
