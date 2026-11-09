#pragma once
#include <cmath>

// Cheap replacements for library math in the audio path (checked in tools/sim/fastmath_check.py).

// tanh(x): Pade [7/6] form, capped at 1 where it reaches 1 (|x| ~ 4.97). Max error ~1e-4.
inline float FastTanh(float x)
{
    const float a = fabsf(x);
    const float a2 = a * a;
    const float p = a * (135135.0f + a2 * (17325.0f + a2 * (378.0f + a2)));
    const float q = 135135.0f + a2 * (62370.0f + a2 * (3150.0f + 28.0f * a2));
    float y = p / q;
    if (y > 1.0f || a > 9.0f) y = 1.0f;
    return (x < 0.0f) ? -y : y;
}

// sin(pi/2 * x) for x in 0 .. 1 (Taylor to t^9). Max error ~4e-6. cos(pi/2 * x) = FastSinHalfPi(1 - x).
inline float FastSinHalfPi(float x)
{
    const float t = 1.5707963f * x;
    const float t2 = t * t;
    return t * (1.0f + t2 * (-1.6666667e-1f + t2 * (8.3333333e-3f + t2 * (-1.9841270e-4f + t2 * 2.7557319e-6f))));
}
