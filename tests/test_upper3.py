import pathlib
import re
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from upper_can import Receiver, fragments


class HostTransportTest(unittest.TestCase):
    def test_documented_vectors(self):
        self.assertEqual(list(fragments("STAT?", 1)), [bytes.fromhex("c0 01 53 54 41 54 3f")])
        self.assertEqual(list(fragments("PA1.234", 2)), [bytes.fromhex("80 02 50 41 31 2e 32 33"), bytes.fromhex("41 02 34")])

    def test_roundtrip(self):
        for text in ("STAT?", "HELLO 5090", "PA1.234", "VPID0.1,0.2,0", "x" * 192):
            rx = Receiver()
            results = [rx.feed(packet, i * 0.001) for i, packet in enumerate(fragments(text, 255))]
            self.assertTrue(all(result is None for result in results[:-1]))
            self.assertEqual(results[-1], text)

    def test_drop_and_timeout(self):
        parts = list(fragments("HELLO 5090 MORE TEXT", 1))
        rx = Receiver()
        self.assertIsNone(rx.feed(parts[0], 0))
        self.assertIsNone(rx.feed(parts[2], .01))
        self.assertIsNone(rx.feed(parts[3], .02))
        self.assertIsNone(rx.feed(parts[0], 1))
        self.assertIsNone(rx.feed(parts[1], 1.6))
        self.assertIsNone(rx.feed(parts[2], 1.61))
        self.assertIsNone(rx.feed(parts[3], 1.62))

    def test_command_boundaries(self):
        for text in ("", "x" * 193):
            with self.assertRaises(ValueError):
                list(fragments(text, 1))


class BoardMigrationTest(unittest.TestCase):
    def test_schematic_pin_mapping(self):
        config = (ROOT / "Config.h").read_text(encoding="utf-8")
        pins = {name: int(value) for name, value in re.findall(r"^#define\s+(PIN_\w+)\s+(\d+)\b", config, re.M)}
        expected = dict(PIN_SPI_SCK=4, PIN_SPI_MOSI=5, PIN_SPI_MISO=6,
                        PIN_ENC_A_CS=17, PIN_ENC_B_CS=38,
                        PIN_A_U=9, PIN_A_V=10, PIN_A_W=11, PIN_A_SLEEP=41, PIN_A_FAULT=42,
                        PIN_B_U=12, PIN_B_V=13, PIN_B_W=14, PIN_B_SLEEP=39, PIN_B_FAULT=40,
                        PIN_VON_ADC=18, PIN_A_CURRENT_U=1, PIN_A_CURRENT_V=2,
                        PIN_B_CURRENT_U=7, PIN_B_CURRENT_V=8, PIN_LED1=21, PIN_LED2=47,
                        PIN_CAN_TX=15, PIN_CAN_RX=16)
        self.assertEqual(pins, expected)
        self.assertEqual(len(pins), len(set(pins.values())))
        self.assertFalse(set(pins.values()) & {35, 36, 37, 45, 46})

    def test_no_removed_axis_references(self):
        for path in list(ROOT.glob("*.ino")) + list(ROOT.glob("*.h")):
            text = path.read_text(encoding="utf-8")
            self.assertNotRegex(text, r"\b(?:motorC|sensorC|driverC|currentSenseC|homeCOffset|PIN_C_\w+|MOTOR_C_\w+)\b")
        for command in ('"STATC?"', '"SLEEPC"', '"WAKEC"', '"ZC!"'):
            self.assertNotIn(command, (ROOT / "Protocol.ino").read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
