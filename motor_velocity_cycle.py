"""Cycle all Upper3.0 motors between +2.0 and -2.0 rad/s on COM21.

Press Space to stop both motors and exit. This is a standalone host-side
utility and is not part of the Arduino firmware build.
"""

from __future__ import annotations

import sys
import time

try:
    import msvcrt
    import serial
except ImportError as exc:
    missing = "pyserial" if exc.name == "serial" else exc.name
    raise SystemExit(f"Missing dependency: {missing}. Run: py -m pip install pyserial") from exc


PORT = "COM21"
BAUD_RATE = 115200
SPEED_RAD_S = 2.0
MOVE_SECONDS = 2.0
POLL_SECONDS = 0.02
AXES = ("A", "B")


def send_all_velocity(port: serial.Serial, velocity: float) -> None:
    """Set both velocity targets with the Upper3.0 text protocol."""
    for axis in AXES:
        command = f"V{axis}{velocity:.1f}\n"
        port.write(command.encode("ascii"))
    port.flush()


def space_pressed() -> bool:
    """Consume pending console keys and report whether Space was pressed."""
    pressed = False
    while msvcrt.kbhit():
        if msvcrt.getwch() == " ":
            pressed = True
    return pressed


def wait_or_stop(port: serial.Serial, duration: float) -> bool:
    """Wait while keeping serial RX drained; return True on Space."""
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        if space_pressed():
            return True

        # Drain firmware replies/heartbeat so the host receive buffer cannot fill.
        waiting = port.in_waiting
        if waiting:
            port.read(waiting)

        time.sleep(min(POLL_SECONDS, max(0.0, deadline - time.monotonic())))
    return False


def stop_motors(port: serial.Serial) -> None:
    """Best-effort zero-speed command for every motor."""
    try:
        send_all_velocity(port, 0.0)
        print("\nStop sent: VA0.0, VB0.0")
    except (OSError, serial.SerialException) as exc:
        print(f"\nWarning: failed to send stop commands: {exc}", file=sys.stderr)


def main() -> int:
    try:
        port = serial.Serial(
            port=PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,
            write_timeout=1,
        )
    except serial.SerialException as exc:
        print(f"Cannot open {PORT}: {exc}", file=sys.stderr)
        return 1

    print(f"Connected to {PORT}. All motors alternate at +/-{SPEED_RAD_S} rad/s every 2 s.")
    print("Press Space to stop and exit (Ctrl+C also stops).")

    try:
        port.write(b"VF6\n")
        port.flush()
        print("Startup command sent: VF6")

        direction = 1.0
        while True:
            velocity = direction * SPEED_RAD_S
            send_all_velocity(port, velocity)
            direction_text = "Forward" if direction > 0 else "Reverse"
            print(f"{direction_text}: {velocity:+.1f} rad/s")

            if wait_or_stop(port, MOVE_SECONDS):
                break
            direction *= -1.0
    except KeyboardInterrupt:
        print("\nCtrl+C received.")
    except (OSError, serial.SerialException) as exc:
        print(f"\nSerial communication error: {exc}", file=sys.stderr)
        return 1
    finally:
        stop_motors(port)
        port.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
