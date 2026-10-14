#pragma once
#include <math.h>
#include <stdint.h>
namespace zerohero {
constexpr float tau = 6.2831853071795864769f;
constexpr float pi = 0.5f * tau;
// ODrive-style positive modulo and signed circular difference.
inline float positive(float value) {
    float result = fmodf(value, tau);
    return result < 0.0f ? result + tau : result;
}
inline float difference(float value) {
    return positive(value + pi) - pi;
}
inline int32_t rotationsAfterSample(float previous, float current, int32_t rotations) {
    float delta = current - previous;
    if (delta > pi) --rotations;
    else if (delta < -pi) ++rotations;
    return rotations;
}
// Matches the official MT6701 SSI driver's MODE2 / first-bit-ignore layout.
inline unsigned short officialAngle(unsigned short word) {
    return (word >> 1) & 0x3FFF;
}
}
