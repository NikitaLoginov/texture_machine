#pragma once
#include <cstddef>
#include <cstdint>
#include "tape.h"

class GrainPlayer {
public:
    enum Mode { FORWARD = 0, PINGPONG = 1, BACKWARD = 2 };

    void Init(const TapeFrame* tape, size_t capacity, float sample_rate);   // tape: capacity frames
    void SetLength(size_t length);     // valid recorded samples, clamp to 0..capacity
    void SetRange(float a, float b);   // loop start a (0..1 of length), b = loop length 0..1 (exponential, 100 ms .. the rest after the start); applied live at the next grain start (see Process), or on Reset()
    void SetMode(Mode mode);
    void SetSpeed(float speed);        // playhead speed, 1 = normal, 0 = freeze, clamp to >= 0
    void SetRatio(float ratio);        // grain read speed (pitch) = tape speed, clamp 0..8 (0 = stopped tape)
    void SetGrainMs(float ms);         // grain length in ms, clamp 5..1000
    void SetFadeMs(float ms);          // loop crossfade length, clamp 1..2000
    void SetWriting(bool writing) { writing_ = writing; }   // the record head writes: hold loop changes near the loop edges
    void SetGlitch(float amount) { glitch_ = (amount < 0.0f) ? 0.0f : (amount > 1.0f ? 1.0f : amount); }   // P06: share of grains that jump to random places (once per block)
    // P07 beat repeat in tape mode (once per block): held, and the slice length in tape samples.
    void SetRepeat(bool hold, double slice) { rep_hold_ = hold; rep_next_len_ = (slice < 2.0 * kRepXf) ? 2.0 * kRepXf : slice; }
    // Printing effects (overdub): while on, Process also stores the plain tape under the playhead
    // (both tracks, no effects), read with PlainL() / PlainR() after Process. Once per block.
    void SetPrint(bool on) { print_ = on; }
    float PlainL() const { return plain_l_; }
    float PlainR() const { return plain_r_; }

    // Lead-in: the input just before the recording, a ring of len samples per channel whose
    // oldest sample is at index phase. Lead-in sample j (-len .. -1, -1 = just before the
    // recording) is at ring index (phase + len + j) % len.
    void SetPreRoll(const float* l, const float* r, size_t len, size_t phase);

    // Overdub track (in the tape frames): read with its own short seam at the loop end
    // (kOvdXfade), not the S33 crossfade. Its lead-in before sample 0: sample m < 0 at index len + m.
    void SetOverdub(const float* pre_l, const float* pre_r, size_t pre_len);
    void Reset();                      // playhead and grains back to region start
    void Process(float& out_l, float& out_r);  // one stereo frame

    // For the overdub record head:
    double Pos() const { return pos_; }                      // playhead inside the region, samples
    float  Step() const { return step_; }                    // signed move in the last Process(), before wrap/bounce
    bool   Bounces() const { return mode_ == PINGPONG; }
    size_t RegionStart() const { return (size_t)region_start_; }
    size_t RegionLen() const { return (size_t)region_len_; }
    long   RegionXfade() const { return Xfade((long)region_len_); }   // crossfade of the current loop, samples
    size_t RegionLead() const { return (size_t)region_start_; }      // the crossfade lead is the audio before this sample
    long   RegionOvdXfade() const { return OvdXfade((long)region_len_); }   // overdub track seam, samples

private:
    void UpdateRegion();                                // apply the loop set by SetRange now
    void NextRegion(double& start, double& len) const;  // the loop set by SetRange
    bool Frozen() const;                                // no loop change now (record head near the loop edges)
    void Apply();                                       // at a grain start: take the loop set by SetRange
    void StartGrain(int g);                             // grain g starts at the playhead, in its loop
    static double Wrap(double x, double len);
    long Xfade(long n) const;
    void Lead(long m, float& out_l, float& out_r) const;
    void LoopSample(long base, long n, long xf, long xo, long k, float& out_l, float& out_r) const;
    void ReadAt(double rel, long base, long n, float& out_l, float& out_r) const;
    long OvdXfade(long n) const;
    void OvdLead(long m, float& out_l, float& out_r) const;
    static constexpr long kOvdXfade = 480;   // overdub track seam at the loop end, samples (10 ms)
    static constexpr double kEdgeHold = 512.0;   // frozen zone margin: the record head's 10 ms tails + its lag

    const TapeFrame* tape_;   // the tape: recording + overdub track per position
    const float* pre_l_;
    const float* pre_r_;
    size_t pre_len_;     // lead-in length, samples (0 = none)
    size_t pre_phase_;   // ring index of the oldest lead-in sample
    bool has_ovd_;            // overdub track in use (SetOverdub called)
    const float* ovd_pre_l_;  // its lead-in before sample 0
    const float* ovd_pre_r_;
    size_t ovd_pre_len_;
    size_t capacity_;
    float sr_;

    size_t length_;
    float range_a_;
    float range_b_;
    double region_start_;   // the playhead's loop
    double region_len_;
    bool writing_;

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
    double gbase_[2];   // per-grain loop (start, length): the playhead's loop when the grain started
    double glen_[2];
    float glitch_;      // P06 glitch amount: chance that a starting grain jumps
    uint32_t seed_;     // random state for the glitch
    float Rand01();     // 0 .. 1

    // Beat repeat reader (P07, tape mode).
    static constexpr double kRepXf = 288.0;           // slice wrap crossfade, tape samples (6 ms)
    static constexpr float kRepRel = 1.0f / 480.0f;   // release crossfade step (10 ms)
    bool rep_hold_, rep_on_, rep_first_;   // held; repeating; still in its first pass (= the playback)
    float rep_g_;                          // repeat share: 0 = grains .. 1 = repeat
    float rep_dir_;                        // playback direction at the press
    double rep_base_, rep_n_;              // the loop at the press (start, length)
    double rep_start_;                     // slice start in that loop
    double rep_q_;                         // read position in the slice, tape samples
    double rep_len_, rep_next_len_;        // slice length now / from the next wrap on
    bool print_;                           // store the plain tape under the playhead
    float plain_l_, plain_r_;              // that sample (this Process)
};
