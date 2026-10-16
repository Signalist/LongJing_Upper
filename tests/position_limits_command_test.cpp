#include <cassert>
#include <iostream>
#include "../position_limits.h"
#include "../libraries/Simple_FOC/src/common/zerohero_angle.h"

int main() {
  float lower = 99, upper = 100;
  assert(position_limits::parse("-7.2,4.3", lower, upper));
  assert(lower == -7.2f && upper == 4.3f);
  assert(position_limits::valid(lower, upper));
  assert(position_limits::contains(-1.3f, lower, upper));
  assert(!position_limits::contains(-7.3f, lower, upper));
  assert(position_limits::contains(lower, lower, upper));
  assert(position_limits::contains(upper, lower, upper));
  assert(position_limits::parse(" -2e0 , +3.5 \t", lower, upper));
  assert(lower == -2 && upper == 3.5f);
  assert(zerohero::boundedPosition(-9, lower, upper) == lower);
  assert(zerohero::boundedPosition(9, lower, upper) == upper);
  assert(zerohero::outwardAtLimit(upper, 1, lower, upper));
  assert(zerohero::outwardAtLimit(lower, -1, lower, upper));
  for (const char* bad : {"", "1", ",2", "1,", "1,,2", "1 2", "1,2,3",
                          "1,2x", "NaN,2", "-inf,2", "1,inf", "1e999,2",
                          "1e-999,2", "--1,2", "1/2"}) {
    lower = 99; upper = 100;
    assert(!position_limits::parse(bad, lower, upper));
    assert(lower == 99 && upper == 100);
  }
  assert(!position_limits::valid(1, 1));
  assert(!position_limits::valid(2, 1));
  assert(!position_limits::valid(NAN, 1));
  assert(!position_limits::valid(-1, INFINITY));
  assert(!position_limits::contains(NAN, -1, 1));
  std::cout << "PASS: ALIMIT parsing, finite/range validation, current-position guard, dynamic clamps\n";
}
