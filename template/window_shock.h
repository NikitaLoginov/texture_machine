#pragma once
#include <cstdint>
#include <cmath>
#include "fast_math.h"

// Pad effect "window shock": a crinkled, damaged stretch of tape under the head while P08 is held
// (severity = pad pressure 0..1). On the tape signal, before the tape sound: dropouts (the level
// dips and the highs drop, 5-40 ms), distortion bursts (10-80 ms, level-matched), crinkly crackle
// (tiny noise bursts in wandering clusters through a soft band-pass, independent of the signal,
// per channel) and capstan hiccups (short speed dips, SpeedFactor). Every event has raised-cosine ramps; at severity 0 no
// new events start and the output equals the input once the running ones have ended.
// Times in samples at 48 kHz. Checked in tools/sim/window_shock_sim.py and crackle2_sim.py.
class WindowShock {
public:
    void Init(float sample_rate)
    {
        inv_sr_ = 1.0f / sample_rate;
        lp_a_ = 1.0f - expf(-6.2831853f * 1500.0f / sample_rate);   // dropout highs loss, 1.5 kHz
        // Crackle: band-pass (RBJ, 0 dB peak) 2.5 kHz, Q 0.7, then a one-pole low-pass at 4 kHz.
        const float w0 = 6.2831853f * 2500.0f / sample_rate;
        const float al = sinf(w0) / (2.0f * 0.7f);
        const float a0 = 1.0f + al;
        b0_ = al / a0;
        b2_ = -al / a0;
        a1_ = -2.0f * cosf(w0) / a0;
        a2_ = (1.0f - al) / a0;
        crk_lp_a_ = 1.0f - expf(-6.2831853f * 4000.0f / sample_rate);
    }

    // Once per block: pad pressure 0..1.
    void SetSeverity(float s) { sev_ = (s < 0.0f) ? 0.0f : (s > 1.0f ? 1.0f : s); }

    // Once per block of n samples: factor on the tape speed (capstan hiccups, 1 = none).
    float SpeedFactor(int n)
    {
        if (hic_len_ == 0) {
            if (Rand01() >= sev_ * kHicRate * (float)n * inv_sr_) return 1.0f;
            hic_len_ = 960 + (int32_t)(Rand01() * 1920.0f);   // 20 .. 60 ms
            hic_pos_ = 0;
            hic_depth_ = sev_ * (0.02f + 0.04f * Rand01());   // 2 .. 6 % slower at most
        }
        const float w = sinf(3.14159265f * (float)hic_pos_ / (float)hic_len_);
        hic_pos_ += n;
        if (hic_pos_ >= hic_len_) hic_len_ = 0;
        return 1.0f - hic_depth_ * w * w;
    }

    // Once per sample, both channels of the tape signal.
    void Process(float& l, float& r)
    {
        // Idle (pad released, every event and the crackle died down): bypass. Only the dropout
        // low-pass keeps following the input, so the next dropout starts clean.
        if (sev_ == 0.0f && !drop_.on && !dist_.on && Settle(crk_l_) && Settle(crk_r_)) {
            lp_l_ = l;
            lp_r_ = r;
            return;
        }
        // Dropout: the level dips and the highs drop.
        if (!drop_.on && Rand01() < sev_ * kDropRate * inv_sr_) {
            drop_.Start(144, 240 + (int32_t)(Rand01() * 1680.0f), 240);   // 3 ms in, 5-40 ms, 5 ms out
            depth_ = sev_ * (0.5f + 0.45f * Rand01());
        }
        const float d = depth_ * drop_.Next();
        lp_l_ += lp_a_ * (l - lp_l_);
        lp_r_ += lp_a_ * (r - lp_r_);
        l = (l + d * (lp_l_ - l)) * (1.0f - d);
        r = (r + d * (lp_r_ - r)) * (1.0f - d);
        // Distortion burst: faded-in extra saturation, a 0.5 peak stays at 0.5.
        if (!dist_.on && Rand01() < sev_ * kDistRate * inv_sr_) {
            dist_.Start(240, 480 + (int32_t)(Rand01() * 3360.0f), 480);   // 5 ms in, 10-80 ms, 10 ms out
            g_ = 1.0f + 7.0f * sev_ * (0.3f + 0.7f * Rand01());
            gnorm_ = 0.5f / tanhf(0.5f * g_);
        }
        const float b = dist_.Next();
        if (b > 0.0f) {
            l += b * (FastTanh(g_ * l) * gnorm_ - l);
            r += b * (FastTanh(g_ * r) * gnorm_ - r);
        }
        // Crackle: tiny noise bursts at a density that wanders in clusters (new target every
        // 150-400 ms, smoothed), independent per channel.
        if (--dens_cnt_ <= 0) {
            const float u = Rand01();
            dens_to_ = 0.1f + 1.9f * u * u;
            dens_cnt_ = 7200 + (int32_t)(Rand01() * 12000.0f);
        }
        dens_ += 0.0005f * (dens_to_ - dens_);
        const float tick = sev_ * kTickRate * dens_ * inv_sr_;
        l += Crackle(crk_l_, tick);
        r += Crackle(crk_r_, tick);
    }

private:
    // Raised-cosine event envelope 0..1: a samples in, h held, r out.
    struct Ramp {
        int32_t pos = 0, a = 1, h = 0, r = 1;
        bool on = false;
        void Start(int32_t in, int32_t hold, int32_t out)
        {
            a = in;
            h = hold;
            r = out;
            pos = 0;
            on = true;
        }
        float Next()
        {
            if (!on) return 0.0f;
            const int32_t p = pos++;
            if (p < a) {
                const float s = FastSinHalfPi((float)p / (float)a);   // sin^2 = 0.5 - 0.5 cos
                return s * s;
            }
            if (p < a + h) return 1.0f;
            if (p < a + h + r) {
                const float c = FastSinHalfPi(1.0f - (float)(p - a - h) / (float)r);   // cos^2 = 0.5 + 0.5 cos
                return c * c;
            }
            on = false;
            return 0.0f;
        }
    };

    static constexpr float kDropRate = 6.0f;    // dropouts per second at full severity (one at a time)
    static constexpr float kDistRate = 2.0f;    // distortion bursts per second at full severity
    static constexpr float kTickRate = 100.0f;  // crackle ticks per second per channel at full severity (x density)
    static constexpr float kHicRate = 3.0f;     // capstan hiccups per second at full severity

    // One channel of crackle: a tick starts a noise burst (random size, 0.3-1.5 ms decay), then
    // the band-pass and the 4 kHz low-pass.
    struct Crk {
        float env = 0.0f, dec = 0.9f, z[2] = {0.0f, 0.0f}, lp = 0.0f;
    };
    float Crackle(Crk& c, float tick)
    {
        if (Rand01() < tick) {
            const float u = Rand01();
            const float a = sev_ * (0.008f + 0.052f * u * u);
            if (a > c.env) c.env = a;
            c.dec = 1.0f - 1.0f / (15.0f + 60.0f * Rand01());
        }
        float x = 0.0f;
        if (c.env > 1e-5f) {
            x = c.env * (2.0f * Rand01() - 1.0f);
            c.env *= c.dec;
        } else {
            c.env = 0.0f;
        }
        c.lp += crk_lp_a_ * (Bp(c.z, x) - c.lp);
        return c.lp;
    }
    // Crackle channel died down (below -180 dB): its filter states are zeroed (in float they
    // would otherwise stall at the smallest subnormal instead of reaching 0).
    static bool Settle(Crk& c)
    {
        if (c.env != 0.0f) return false;
        if (fabsf(c.lp) > 1e-9f || fabsf(c.z[0]) > 1e-9f || fabsf(c.z[1]) > 1e-9f) return false;
        c.lp = c.z[0] = c.z[1] = 0.0f;
        return true;
    }
    float Bp(float* z, float x)   // band-pass, transposed direct form II (b1 = 0)
    {
        const float y = b0_ * x + z[0];
        z[0] = -a1_ * y + z[1];
        z[1] = b2_ * x - a2_ * y;
        return y;
    }
    float Rand01()   // 0 .. 1
    {
        seed_ ^= seed_ << 13;
        seed_ ^= seed_ >> 17;
        seed_ ^= seed_ << 5;
        return (float)(seed_ >> 8) * (1.0f / 16777216.0f);
    }

    uint32_t seed_ = 0x2545F491u;
    float inv_sr_ = 1.0f / 48000.0f;
    float sev_ = 0.0f;
    Ramp  drop_, dist_;
    float depth_ = 0.0f, lp_a_ = 0.18f, lp_l_ = 0.0f, lp_r_ = 0.0f;
    float g_ = 1.0f, gnorm_ = 1.0f;
    float b0_ = 0.0f, b2_ = 0.0f, a1_ = 0.0f, a2_ = 0.0f;
    Crk   crk_l_, crk_r_;
    float crk_lp_a_ = 0.4f;   // crackle low-pass coefficient
    float dens_ = 0.7f, dens_to_ = 0.7f;   // crackle density factor and its target
    int32_t dens_cnt_ = 0;                 // samples until the next density target
    int32_t hic_len_ = 0, hic_pos_ = 0;
    float hic_depth_ = 0.0f;
};
