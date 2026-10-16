#include <cassert>
#include <cmath>
#include <iostream>
#include "../libraries/Simple_FOC/src/common/zerohero_angle.h"

// Injected from the actual Config.h by the test command.
int main() {
    using namespace zerohero;
    const float lower = TEST_A_MIN;
    const float upper = TEST_A_MAX;
    assert(std::fabs(lower + 7.2f) < 0.00001f);
    assert(std::fabs(upper - 4.3f) < 0.00001f);
    assert(boundedPosition(-8, lower, upper) == lower);
    assert(boundedPosition(8, lower, upper) == upper);
    assert(boundedPosition(4.0f, lower, upper) == 4.0f);
    assert(boundedPosition(-6.2831853f, lower, upper) == -6.2831853f);
    assert(boundedPosition(-4.2f, lower, upper) == -4.2f);
    assert(outwardAtLimit(upper, 1, lower, upper));
    assert(outwardAtLimit(lower, -1, lower, upper));
    assert(!outwardAtLimit(upper, -1, lower, upper));
    assert(!outwardAtLimit(lower, 1, lower, upper));
    assert(boundedPosition(4.2f + 0.3f, lower, upper) == upper);
    assert(boundedPosition(-7.1f - 0.3f, lower, upper) == lower);
    assert(std::fabs(TEST_B_MIN + tau) < 0.00001f);
    assert(std::fabs(TEST_B_MAX - tau) < 0.00001f);
    std::cout << "PASS: A[-7.2,4.3], position/displacement clamps, outward guards, B unchanged\n";
}
