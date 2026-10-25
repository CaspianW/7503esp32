#!/usr/bin/env python3
"""@file keyboard_control_test.py
@brief 键盘输入丢失时验证手动控制立即停机并要求重新使能。
@author Caspian Wu
@date 2026-10-24
"""

import importlib.util
import pathlib
import sys
import types
import unittest
from unittest.mock import patch


ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "keyboard_control", ROOT / "tools" / "keyboard_control.py"
)
keyboard_control = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(keyboard_control)


class KeyboardControlTest(unittest.TestCase):
    def test_dropped_input_stops_and_requires_rearm(self):
        ecodes = types.SimpleNamespace(
            EV_SYN=0,
            EV_KEY=1,
            SYN_DROPPED=3,
            KEY_W=17,
            KEY_S=31,
            KEY_A=30,
            KEY_D=32,
            KEY_Q=16,
            KEY_E=18,
            KEY_SPACE=57,
            KEY_R=19,
            KEY_T=20,
            KEY_ESC=1,
            KEY_U=22,
            KEY_J=36,
            KEY_MINUS=12,
            KEY_EQUAL=13,
        )

        class FakeKeyboard:
            fd = 10
            path = "/dev/input/event-test"
            name = "fake keyboard"

            def __init__(self):
                self.events = iter(
                    [
                        types.SimpleNamespace(type=ecodes.EV_KEY, code=ecodes.KEY_W, value=1),
                        types.SimpleNamespace(type=ecodes.EV_SYN, code=ecodes.SYN_DROPPED, value=0),
                        types.SimpleNamespace(type=ecodes.EV_KEY, code=ecodes.KEY_R, value=1),
                        types.SimpleNamespace(type=ecodes.EV_KEY, code=ecodes.KEY_U, value=1),
                        types.SimpleNamespace(type=ecodes.EV_KEY, code=ecodes.KEY_J, value=1),
                        types.SimpleNamespace(type=ecodes.EV_KEY, code=ecodes.KEY_W, value=1),
                        types.SimpleNamespace(type=ecodes.EV_KEY, code=ecodes.KEY_T, value=1),
                        types.SimpleNamespace(type=ecodes.EV_KEY, code=ecodes.KEY_T, value=0),
                        types.SimpleNamespace(type=ecodes.EV_KEY, code=ecodes.KEY_T, value=1),
                        types.SimpleNamespace(type=ecodes.EV_KEY, code=ecodes.KEY_T, value=0),
                        types.SimpleNamespace(type=ecodes.EV_KEY, code=ecodes.KEY_R, value=1),
                        types.SimpleNamespace(type=ecodes.EV_KEY, code=ecodes.KEY_ESC, value=1),
                    ]
                )

            def capabilities(self):
                return {ecodes.EV_KEY: [
                    ecodes.KEY_W, ecodes.KEY_S, ecodes.KEY_A, ecodes.KEY_D,
                    ecodes.KEY_Q, ecodes.KEY_E, ecodes.KEY_SPACE, ecodes.KEY_R,
                    ecodes.KEY_ESC,
                    ecodes.KEY_U, ecodes.KEY_J, ecodes.KEY_MINUS,
                    ecodes.KEY_EQUAL, ecodes.KEY_T,
                ]}

            def read(self):
                try:
                    return [next(self.events)]
                except StopIteration:
                    return []

            def close(self):
                pass

        class FakeSerial:
            is_open = True

            def __init__(self, *args, **kwargs):
                self.commands = []
                self.replies = []

            @property
            def in_waiting(self):
                return sum(map(len, self.replies))

            def write(self, data):
                command = data.decode("ascii").strip()
                self.commands.append(command)
                self.replies.append(b"OK\n")

            def readline(self):
                return self.replies.pop(0) if self.replies else b""

            def reset_input_buffer(self):
                self.replies.clear()

            def close(self):
                self.is_open = False

        keyboard = FakeKeyboard()
        connection = FakeSerial()
        serial_module = types.SimpleNamespace(
            Serial=lambda *args, **kwargs: connection,
            SerialException=OSError,
        )
        evdev_module = types.SimpleNamespace(
            InputDevice=lambda path: keyboard,
            ecodes=ecodes,
            list_devices=lambda: [keyboard.path],
        )

        args = types.SimpleNamespace(port="/dev/ttyUSB0", baud=115200, device=None)
        with patch.object(keyboard_control, "parse_args", return_value=args), \
             patch.dict(sys.modules, {"serial": serial_module, "evdev": evdev_module}), \
             patch("select.select", return_value=([keyboard.fd], [], [])), \
             patch.object(keyboard_control.time, "sleep", return_value=None):
            self.assertEqual(keyboard_control.main(), 0)

        commands = connection.commands
        first_motion = commands.index("manual vel 1.0000 0.0000 0.0000")
        dropped_stop = commands.index("manual stop")
        rearm = commands.index("manual arm", commands.index("manual arm") + 1)
        second_motion = commands.index("manual vel 1.0000 0.0000 0.0000", first_motion + 1)

        self.assertLess(first_motion, dropped_stop)
        self.assertLess(dropped_stop, rearm)
        self.assertLess(rearm, second_motion)
        estop_on = commands.index("estop 1")
        estop_off = commands.index("estop 0")
        self.assertLess(second_motion, estop_on)
        self.assertLess(estop_on, estop_off)
        self.assertLess(estop_off, commands.index("manual arm", estop_off))
        self.assertIn("lift up", commands)
        self.assertIn("lift down", commands)
        self.assertFalse(any(
            command.startswith("manual vel ") and command != "manual vel 0 0 0"
            for command in commands[dropped_stop + 1:rearm]
        ))


if __name__ == "__main__":
    unittest.main()
