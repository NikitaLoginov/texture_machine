#pragma once

// The tape: one frame per position with both tracks (the recording and the overdub track) and
// both channels next to each other in memory, so the player and the record head reach all four
// values of a position with one memory access instead of four.
struct TapeFrame {
    float rec_l, rec_r;   // recording
    float ovd_l, ovd_r;   // overdub track
};

// Track 0 = recording, 1 = overdub track; channel 0 = left, 1 = right.
inline float& TapeValue(TapeFrame& f, int track, int ch) { return (&f.rec_l)[2 * track + ch]; }
inline float TapeValue(const TapeFrame& f, int track, int ch) { return (&f.rec_l)[2 * track + ch]; }
