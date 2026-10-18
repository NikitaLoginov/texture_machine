#pragma once
#include <cstddef>

class GrainPlayer {
public:
    enum Mode { FORWARD = 0, PINGPONG = 1, BACKWARD = 2 };

    void Init(const float* buf_l, const float* buf_r, size_t capacity, float sample_rate);
    void SetLength(size_t length);     // valid recorded samples, clamp to 0..capacity
    void SetRange(float a, float b);   // loop region as fractions 0..1 of length, any order
    void SetMode(Mode mode);
    void SetSpeed(float speed);        // playhead speed, 1 = normal, 0 = freeze, clamp to >= 0
    void SetPitch(float semitones);    // grain transposition, clamp -24..24
    void SetGrainMs(float ms);         // grain length in ms, clamp 5..1000
    void SetFadeMs(float ms);          // loop-edge fade in/out length, clamp 1..2000
    void Reset();                      // playhead and grains back to region start
    void Process(float& out_l, float& out_r);  // one stereo frame

    // For the overdub record head:
    double Pos() const { return pos_; }                      // playhead inside the region, samples
    float  Step() const { return step_; }                    // signed move in the last Process(), before wrap/bounce
    bool   Bounces() const { return mode_ == PINGPONG; }
    size_t RegionStart() const { return (size_t)region_start_; }
    size_t RegionLen() const { return (size_t)region_len_; }

private:
    void UpdateRegion();
    static double Wrap(double x, double len);

    const float* buf_l_;
    const float* buf_r_;
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
};
