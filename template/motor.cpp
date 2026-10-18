#include "motor.h"
#include <cmath>

namespace {
const float kKp = 20.0f;
const float kKi = 400.0f;
const float kTau = 0.08f;    // motor inertia, s
const float kTwoPi = 6.28318530718f;
}

void Motor::Init(float update_rate)
{
    dt_ = 1.0f / update_rate;
    battery_ = 0.5f;
    wobble_ = 0.0f;
    speed_ = 1.0f;
    integ_ = 1.15f;
    deficit_ = 0.0f;
    wow_phase_ = 0.0f;
    flutter_phase_ = 0.0f;
    rnd_phase_ = 0.0f;
    rnd_from_ = 0.0f;
    rnd_to_ = 0.0f;
    seed_ = 12345u;
}

float Motor::SmoothRandom()
{
    rnd_phase_ += 2.5f * dt_;
    if (rnd_phase_ >= 1.0f) {
        rnd_phase_ -= 1.0f;
        rnd_from_ = rnd_to_;
        seed_ = seed_ * 1664525u + 1013904223u;                       // LCG
        rnd_to_ = (float)(seed_ >> 8) * (2.0f / 16777216.0f) - 1.0f;  // -1..1
    }
    const float t = rnd_phase_ * rnd_phase_ * (3.0f - 2.0f * rnd_phase_);  // smoothstep
    return rnd_from_ + (rnd_to_ - rnd_from_) * t;
}

float Motor::Process()
{
    auto clamp = [](float x, float lo, float hi) {
        return x < lo ? lo : (x > hi ? hi : x);
    };

    // Battery voltage: 0.3 (dead) .. 1.0 (fresh, knob 0.5) .. 1.5 (over-voltage).
    const float v   = (battery_ <= 0.5f) ? 0.3f + 1.4f * battery_ : 1.0f + (battery_ - 0.5f);
    const float cap = 1.3f * v;                        // most drive the battery can give
    // Tape load: mean 0.15, slowly and randomly varying by +-0.05.
    const float load = 0.15f + 0.05f * SmoothRandom();
    // Over-voltage: from v = 1.2 to 1.5 the regulator gives way; its setpoint rises from
    // normal speed to what the full battery drive would give.
    const float fail   = clamp((v - 1.2f) / 0.3f, 0.0f, 1.0f);
    const float target = 1.0f + fail * (cap - 0.15f - 1.0f);
    // Governor: PI controller toward target, limited by the battery.
    const float err = target - speed_;
    integ_ = clamp(integ_ + kKi * err * dt_, 0.0f, cap);
    const float drive = clamp(kKp * err + integ_, 0.0f, cap);
    // Motor inertia: settles at speed = drive - load.
    speed_ += (drive - load - speed_) * dt_ / kTau;
    if (speed_ < 0.0f) speed_ = 0.0f;
    // Battery weakness: 0 for v >= 0.9, 1 at the dead end.
    deficit_ = clamp((0.9f - v) / 0.6f, 0.0f, 1.0f);
    // Tape-path wow (0.7 Hz) and flutter (8 Hz), stronger when the battery is weak.
    wow_phase_ += 0.7f * dt_;
    if (wow_phase_ >= 1.0f) wow_phase_ -= 1.0f;
    flutter_phase_ += 8.0f * dt_;
    if (flutter_phase_ >= 1.0f) flutter_phase_ -= 1.0f;
    const float depth = wobble_ * (1.0f + 4.0f * deficit_);
    const float wf = depth * (sinf(kTwoPi * wow_phase_) + 0.5f * sinf(kTwoPi * flutter_phase_));
    return speed_ * (1.0f + wf);
}
