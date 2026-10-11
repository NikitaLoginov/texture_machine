#include "daisy_seed.h"
#include "daisysp.h"
#include "touch/touch.h"

using namespace daisy;
using namespace daisysp;
using namespace synthux;

static DaisySeed hw;
static Touch     touch;

void AudioCallback(AudioHandle::InputBuffer in, 
                   AudioHandle::OutputBuffer out, 
                   size_t size) {
    for (size_t i = 0; i < size; i++) {
        // Audio passthrough (replace with your DSP)
        out[0][i] = in[0][i];
        out[1][i] = in[1][i];
    }
}

int main(void) {
    hw.Init(true);
    hw.SetAudioBlockSize(16);
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

    touch.Init(hw);

    hw.StartAudio(AudioCallback);

    // Settle baseline capacitance at boot
    for (int i = 0; i < 40; i++) {
        touch.Process();
        System::Delay(3);
    }

    while (1) {
        touch.Process();

        // Visual feedback: LED turns on when any pad is touched
        hw.SetLed(touch.pads().HasTouch());

        System::Delay(3);
    }
}
