#pragma once
#include <cstdint>

// Cassette transport: speed governor powered by a (dying) battery.
// Call Process() once per audio block. Returns the tape speed factor (1 = normal).
class Motor {
public:
    void  Init(float update_rate);   // Process() calls per second (audio callback rate)
    void  SetBattery(float b) { battery_ = b; }    // 0 = dead, 0.5 = fresh, 1 = over-voltage
    void  SetWobble(float d)  { wobble_ = d; }     // tape wow/flutter depth, e.g. 0.0008
    float Process();                               // about 0.25 .. 1.8
    float Deficit() const { return deficit_; }     // battery weakness, 0 = fine .. 1 = dead

private:
    float SmoothRandom();            // -1..1, smooth, new target 2.5 times per second
    float dt_ = 0.0f;
    float battery_ = 0.5f, wobble_ = 0.0f;
    float speed_ = 1.0f, integ_ = 1.15f, deficit_ = 0.0f;
    float wow_phase_ = 0.0f, flutter_phase_ = 0.0f;
    float rnd_phase_ = 0.0f, rnd_from_ = 0.0f, rnd_to_ = 0.0f;
    uint32_t seed_ = 12345u;
};
