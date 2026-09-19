#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Byte 0: START (bit 7), END (bit 6), sequence modulo 64.
// Byte 1: message number. Bytes 2..7: text, without a terminator.
// A missing/duplicate/out-of-order fragment invalidates the entire command.
template <size_t Capacity> class CanTextReceiver {
 public:
  char text[Capacity + 1] = {};
  size_t length = 0;
  bool active = false;
  uint8_t message = 0, sequence = 0;
  uint32_t lastMs = 0;
  void reset() { active = false; length = 0; text[0] = 0; }
  bool expired(uint32_t now, uint32_t timeout) const {
    return active && uint32_t(now - lastMs) >= timeout;
  }
  // -1 invalid, 0 incomplete, 1 complete (text valid until next feed/reset).
  int feed(const uint8_t* data, size_t dlc, uint32_t now) {
    if (dlc < 3 || dlc > 8) { reset(); return -1; }
    const uint8_t seq = data[0] & 0x3f;
    if (data[0] & 0x80) {
      reset();
      if (seq != 0) return -1;
      active = true; message = data[1]; sequence = 0;
    }
    if (!active || message != data[1] || sequence != seq || length + dlc - 2 > Capacity) {
      reset(); return -1;
    }
    for (size_t i = 2; i < dlc; ++i) {
      if (data[i] == 0) { reset(); return -1; }
      text[length++] = char(data[i]);
    }
    text[length] = 0;
    sequence = (sequence + 1) & 0x3f;
    lastMs = now;
    if (data[0] & 0x40) { active = false; return 1; }
    return 0;
  }
};
