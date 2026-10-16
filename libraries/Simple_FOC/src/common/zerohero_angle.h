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
// A new boot knows only the physical single-turn home, not historical turns.
// Select that home's nearest equivalent once; never rebase it during operation.
inline float nearestHome(float shaft, float storedHome) {
    return shaft - difference(shaft - storedHome);
}
inline float boundedPosition(float value, float lower, float upper) {
    return fminf(upper, fmaxf(lower, value));
}
inline bool outwardAtLimit(float position, float demand, float lower, float upper) {
    return (position >= upper && demand > 0) ||
           (position <= lower && demand < 0);
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
