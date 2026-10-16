#include <cassert>
#include <cmath>
#include <iostream>
#include "../libraries/Simple_FOC/src/common/zerohero_angle.h"

static void near(float a, float b, float tolerance = 0.0001f) {
    assert(std::fabs(a - b) < tolerance);
}

int main() {
    using namespace zerohero;
    // Old NVS homes may contain turns. They must select the same physical home.
    for (float physicalHome : {0.0f, 1.308486f, -8.905525f}) {
        for (int sensorBootTurn = -3; sensorBootTurn <= 3; ++sensorBootTurn) {
            const float shaft = sensorBootTurn * tau + 0.04f;
            const float home = nearestHome(shaft, physicalHome);
            assert(std::fabs(shaft - home) <= pi + 0.0001f);
            near(positive(home), positive(physicalHome));
            near(shaft - home, difference(0.04f - physicalHome));
        }
    }
    // Keep a fixed RAM home: never wrap feedback or target as rotation proceeds.
    const float home = nearestHome(6.27f, 0.0f);
    near(6.27f - home, -0.0131853f);
    near(6.2931853f - home, 0.01f);
    near((home + tau) - home, tau);
    near((home - tau) - home, -tau);
    // A +1-turn goal from -1 turn requires TWO turns, not a zero circular error.
    near((home + tau) - (home - tau), 2 * tau);
    near(boundedPosition(100, -tau, tau), tau);
    near(boundedPosition(-100, -tau, tau), -tau);
    near(boundedPosition(tau - 0.1f + 0.3f, -tau, tau), tau);
    near(boundedPosition(-tau + 0.1f - 0.3f, -tau, tau), -tau);
    near(boundedPosition(-0.05f, -tau, tau), -0.05f);
    assert(outwardAtLimit(tau, 0.1f, -tau, tau));
    assert(outwardAtLimit(-tau, -0.1f, -tau, tau));
    assert(!outwardAtLimit(tau, -0.1f, -tau, tau));
    assert(!outwardAtLimit(-tau, 0.1f, -tau, tau));
    assert(!outwardAtLimit(0, 1, -tau, tau));
    // ZERO! at any turn uses current shaft as RAM origin, persists only phase.
    const float zeroShaft = 4 * tau + 1.2f;
    const float stored = positive(zeroShaft);
    near(zeroShaft - zeroShaft, 0);
    near(nearestHome(0.2f, stored), 1.2f);
    // REBASE +1 is a coordinate-only shift: identical physical target/phase.
    const float oldHome = 1.3f;
    const float oldPosition = -0.25f;
    const float newHome = oldHome - tau;
    const float newPosition = oldPosition + tau;
    near(oldHome + oldPosition, newHome + newPosition);
    near(positive(oldHome), positive(newHome));
    assert(newPosition >= -tau && newPosition <= tau);
    near((newPosition - tau) - newPosition, -tau);
    // Raw zero crossings still track continuous angle in both directions.
    assert(rotationsAfterSample(6.27f, 0.01f, 0) == 1);
    assert(rotationsAfterSample(0.01f, 6.27f, 1) == 0);
    std::cout << "PASS: boot home, legacy NVS, signed continuous feedback, +/-1 turn, "
                 "2-turn traversal, clamp, outward limits, ZERO, restart, crossings\n";
}
