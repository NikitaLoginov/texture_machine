#include "tape_echo.h"
#include <cmath>
using namespace daisysp;

void TapeEcho::Init(float sample_rate)
{
    sr_ = sample_rate;
    tape_.Init();
    echo_.Init();
    wow_.Init(sr_);
    wow_.SetWaveform(Oscillator::WAVE_SIN);
    wow_.SetFreq(0.6f);
    wow_.SetAmp(1.5f);
    flutter_.Init(sr_);
    flutter_.SetWaveform(Oscillator::WAVE_SIN);
    flutter_.SetFreq(7.0f);
    flutter_.SetAmp(0.4f);
    echo_delay_ = 0.25f * sr_;
}

float TapeEcho::Process(float in)
{
    // Tape: 20 ms delay wobbling by 1.5 ms at 0.6 Hz and 0.4 ms at 7 Hz.
    const float delay_ms = 20.0f + wow_.Process() + flutter_.Process();
    const float t = tape_.ReadHermite(delay_ms * 0.001f * sr_);
    tape_.Write(in);
    // Saturation.
    const float sat = 0.4f * tanhf(3.0f * t);
    // Echo: 250 ms, feedback 0.4. The sum is both the output and what is fed back.
    const float y = sat + 0.4f * echo_.Read(echo_delay_);
    echo_.Write(y);
    return y;
}
