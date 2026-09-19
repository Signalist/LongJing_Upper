"""Upper3.0 framed classic-CAN console; requires python-can.

Example: python upper_can.py --interface slcan --channel COM21 'STAT?'
The interface/channel must describe a python-can compatible adapter. The
Damiao USB bridge's proprietary serial envelope is not slcan.
"""
from __future__ import annotations
import argparse
import time


def fragments(text: str, message: int):
    payload = text.encode("utf-8")
    if not 1 <= len(payload) <= 192:
        raise ValueError("Command must contain 1..192 UTF-8 bytes")
    for offset in range(0, len(payload), 6):
        chunk = payload[offset:offset + 6]
        flags = ((offset // 6) & 63) | (0x80 if offset == 0 else 0)
        if offset + len(chunk) == len(payload):
            flags |= 0x40
        yield bytes((flags, message & 255)) + chunk


class Receiver:
    def __init__(self):
        self.active = False
        self.data = bytearray()
        self.message = self.sequence = 0
        self.last = 0.0

    def feed(self, data: bytes, now: float):
        if self.active and now - self.last >= 0.5:
            self.active = False
        if not 3 <= len(data) <= 8:
            self.active = False
            return None
        seq = data[0] & 63
        if data[0] & 0x80:
            self.active = seq == 0
            self.data.clear()
            self.message, self.sequence = data[1], 0
        if (not self.active or data[1] != self.message or seq != self.sequence
                or len(self.data) + len(data) - 2 > 2048 or 0 in data[2:]):
            self.active = False
            return None
        self.data.extend(data[2:])
        self.sequence = (seq + 1) & 63
        self.last = now
        if data[0] & 0x40:
            self.active = False
            return self.data.decode("utf-8", errors="replace")
        return None


def main():
    import can
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--channel", required=True)
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--rx-id", type=lambda s: int(s, 0), default=0x591)
    parser.add_argument("--tx-id", type=lambda s: int(s, 0), default=0x590)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("command")
    args = parser.parse_args()
    packets = list(fragments(args.command, int(time.monotonic() * 1000)))
    with can.Bus(interface=args.interface, channel=args.channel, bitrate=args.bitrate,
                 can_filters=[{"can_id": args.rx_id, "can_mask": 0x7ff, "extended": False}]) as bus:
        for data in packets:
            bus.send(can.Message(arbitration_id=args.tx_id, is_extended_id=False, data=data), timeout=0.1)
        receiver = Receiver()
        deadline = time.monotonic() + args.timeout
        received = False
        while time.monotonic() < deadline:
            frame = bus.recv(timeout=min(0.1, max(0, deadline - time.monotonic())))
            if frame is None or frame.is_extended_id or frame.is_remote_frame or frame.is_error_frame or frame.is_fd:
                continue
            text = receiver.feed(frame.data, time.monotonic())
            if text is not None:
                print(text)
                received = True
        if not received:
            raise SystemExit("No complete CAN reply received")


if __name__ == "__main__":
    main()
