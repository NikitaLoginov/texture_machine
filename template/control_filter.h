#pragma once

#include <cstdint>
#include <cmath>

// Removes ADC jitter from a 0..1 control: the output holds still until the input moves more
// than kBand away, then follows it. Rescaled so 0 and 1 stay reachable.
class ControlFilter {
public:
    static constexpr float kBand = 0.004f;   // 0.4 % of travel; tune by ear

    float Process(float in) {
        if (!init_) { held_ = in; init_ = true; }
        if (in > held_ + kBand)      held_ = in - kBand;
        else if (in < held_ - kBand) held_ = in + kBand;
        float out = (held_ - kBand) / (1.0f - 2.0f * kBand);
        if (out < 0.0f) out = 0.0f;
        else if (out > 1.0f) out = 1.0f;
        return out;
    }

private:
    float held_ = 0.0f;
    bool  init_ = false;
};

// Holds a control's applied value until its input has been still for kSettleMs, then takes
// the new value. One object per control, so each has its own timer.
class SettleFilter {
public:
    static constexpr uint32_t kSettleMs = 150;

    // in: control value (already through ControlFilter); now_ms: System::GetNow().
    float Process(float in, uint32_t now_ms) {
        if (!init_) { pending_ = out_ = in; moved_ms_ = now_ms; init_ = true; }
        if (in != pending_) { pending_ = in; moved_ms_ = now_ms; }
        else if (now_ms - moved_ms_ >= kSettleMs) out_ = pending_;
        return out_;
    }

private:
    float    pending_ = 0.0f;   // latest input, waiting to settle
    float    out_ = 0.0f;       // applied value
    uint32_t moved_ms_ = 0;     // time of the last input change
    bool     init_ = false;
};

// Soft takeover for a knob shared between two jobs: the value only follows the knob once the
// knob has reached it (within 0.01) or passed it since the job became active again.
class Pickup {
public:
    // value: start value. caught: follow the knob from the start (else catch it first).
    Pickup(float value, bool caught) : value_(value), caught_(caught) {}

    void  Release() { caught_ = false; armed_ = false; }   // the knob goes to the other job
    float Value() const { return value_; }

    float Process(float knob) {
        if (!caught_) {
            const bool crossed = armed_ && ((last_ <= value_ && knob >= value_) ||
                                           (last_ >= value_ && knob <= value_));
            if (crossed || fabsf(knob - value_) < 0.01f) caught_ = true;
            last_ = knob;
            armed_ = true;
        }
        if (caught_) value_ = knob;
        return value_;
    }

private:
    float value_;
    float last_ = 0.0f;
    bool  caught_;
    bool  armed_ = false;
};
