#pragma once

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
