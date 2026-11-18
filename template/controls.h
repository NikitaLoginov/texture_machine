#pragma once
// Controls (controls.cpp): the Touch 2's knobs, faders, switches and pads -> the values in app.h.
void SetupControls();   // before hw.StartAudio: touch hardware + pad calibration
void StartControls();   // after hw.StartAudio: settle the pads, pad press handler (P03, P10, P11)
void ReadControls();    // main loop: read everything, set the audio callback's values, the LED
