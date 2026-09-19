#include "../CanTransport.h"
#include <assert.h>
#include <stdio.h>

int main() {
  CanTextReceiver<192> rx;
  uint8_t first[] = {0x80, 7, 'P', 'A', '1', '.', '2', '3'};
  uint8_t last[] = {0x41, 7, '4'};
  assert(rx.feed(first, 8, 10) == 0);
  assert(rx.feed(last, 3, 11) == 1);
  assert(strcmp(rx.text, "PA1.234") == 0);
  assert(rx.feed(last, 3, 12) == -1); // duplicate end cannot execute twice
  assert(rx.feed(first, 8, 20) == 0);
  last[0] = 0x42;
  assert(rx.feed(last, 3, 21) == -1); // lost middle fragment
  last[0] = 0x41;
  assert(rx.feed(last, 3, 22) == -1); // suffix cannot become a command
  assert(rx.feed(first, 8, 30) == 0);
  last[1] = 8;
  assert(rx.feed(last, 3, 31) == -1); // another sender/message
  uint8_t one[] = {0xc0, 9, 'S', 'T', 'A', 'T', '?'};
  assert(rx.feed(one, 7, 40) == 1);
  assert(strcmp(rx.text, "STAT?") == 0);
  one[2] = 0;
  assert(rx.feed(one, 7, 41) == -1);
  assert(rx.feed(first, 2, 42) == -1);
  assert(rx.feed(first, 9, 43) == -1);
  assert(rx.feed(first, 8, 0xfffffff0u) == 0);
  assert(!rx.expired(0x20u, 100));
  assert(rx.expired(0x54u, 100)); // millis wraparound
  rx.reset();
  assert(rx.feed(last, 3, 0x55u) == -1);
  CanTextReceiver<6> small;
  assert(small.feed(first, 8, 0) == 0);
  last[1] = 7;
  assert(small.feed(last, 3, 1) == -1); // overflow discards entire message
  CanTextReceiver<500> large;
  uint8_t chunk[] = {0x80, 2, 'x', 'x', 'x', 'x', 'x', 'x'};
  for (int i = 0; i < 70; ++i) {
    chunk[0] = (i & 63) | (i == 0 ? 0x80 : 0) | (i == 69 ? 0x40 : 0);
    assert(large.feed(chunk, 8, i) == (i == 69 ? 1 : 0));
  }
  assert(large.length == 420); // sequence wrap for long replies
  puts("CAN transport tests passed");
}
