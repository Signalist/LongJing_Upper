#pragma once
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cctype>

namespace position_limits {
inline bool valid(float lower, float upper) {
  return std::isfinite(lower) && std::isfinite(upper) && lower < upper;
}
inline bool contains(float position, float lower, float upper) {
  return std::isfinite(position) && position >= lower && position <= upper;
}
// Exactly two finite decimal/scientific numbers, separated by one comma.
inline bool parse(const char* text, float& lower, float& upper) {
  float values[2];
  for (int i = 0; i < 2; ++i) {
    while (std::isspace(static_cast<unsigned char>(*text))) ++text;
    if (!std::isdigit(static_cast<unsigned char>(*text)) &&
        *text != '+' && *text != '-' && *text != '.') return false;
    errno = 0;
    char* end = nullptr;
    values[i] = std::strtof(text, &end);
    if (end == text || errno == ERANGE || !std::isfinite(values[i])) return false;
    text = end;
    while (std::isspace(static_cast<unsigned char>(*text))) ++text;
    if (i == 0) {
      if (*text != ',') return false;
      ++text;
    } else if (*text != '\0') return false;
  }
  lower = values[0];
  upper = values[1];
  return true;
}
}
