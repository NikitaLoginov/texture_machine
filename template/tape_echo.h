#pragma once
#include "daisysp.h"

// Port of the plugdata tape + echo, one channel. Returns the wet signal.
class TapeEcho {
public:
    void  Init(float sample_rate);
    float Process(float in);
private:
    daisysp::DelayLine<float, 2048>  tape_;   // max ~42 ms; tape delay is 18..22 ms
    daisysp::DelayLine<float, 12288> echo_;   // echo reads 250 ms = 12000 samples at 48 kHz
    daisysp::Oscillator wow_, flutter_;
    float sr_;
    float echo_delay_;                        // samples
};
