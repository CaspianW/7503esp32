#!/usr/bin/env python3
"""@file keyboard_control_test.py
@brief Verify dropped keyboard events stop motion and require rearming.
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
    @staticmethod
    def _base_ecodes(**extra):
        base = dict(
            EV_SYN=0, EV_KEY=1, SYN_DROPPED=3,
            KEY_W=17, KEY_S=31, KEY_A=30, KEY_D=32, KEY_Q=16, KEY_E=18,
            KEY_SPACE=57, KEY_R=19, KEY_T=20, KEY_ESC=1,
            KEY_U=22, KEY_J=36, KEY_MINUS=12, KEY_EQUAL=13,
        )
        base.update(extra)
        return types.SimpleNamespace(**base)

    @staticmethod
    def _run_session(ecodes, make_events):
        cap_keys = [value for name, value in vars(ecodes).items() if name.startswith("KEY_")]

        class FakeKeyboard:
            fd = 10
            path = "/dev/input/event-test"
            name = "fake keyboard"

            def __init__(self):
                self.events = iter(make_events(ecodes))

            def capabilities(self):
                return {ecodes.EV_KEY: cap_keys}

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
                self.commands.append(data.decode("ascii").strip())
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
            status = keyboard_control.main()
        return status, connection.commands

    def test_optional_odometry_and_imu_logs(self):
        self.assertTrue(keyboard_control.should_log("S 100 1", False, False))
        self.assertTrue(keyboard_control.should_log("E 0.1", False, False))
        self.assertFalse(keyboard_control.should_log("O 100 1.0", False, False))
        self.assertFalse(keyboard_control.should_log("I 100 0.0", False, False))
        self.assertTrue(keyboard_control.should_log("O 100 1.0", True, False))
        self.assertFalse(keyboard_control.should_log("I 100 0.0", True, False))
        self.assertTrue(keyboard_control.should_log("I 100 0.0", False, True))
        self.assertFalse(keyboard_control.should_log("O 100 1.0", False, True))
        self.assertFalse(keyboard_control.should_log("OK", True, True))

    def test_dropped_input_stops_and_requires_rearm(self):
        ecodes = self._base_ecodes()
        status, commands = self._run_session(ecodes, lambda e: [
            types.SimpleNamespace(type=e.EV_KEY, code=e.KEY_W, value=1),
            types.SimpleNamespace(type=e.EV_SYN, code=e.SYN_DROPPED, value=0),
            types.SimpleNamespace(type=e.EV_KEY, code=e.KEY_R, value=1),
            types.SimpleNamespace(type=e.EV_KEY, code=e.KEY_U, value=1),
            types.SimpleNamespace(type=e.EV_KEY, code=e.KEY_U, value=2),
            types.SimpleNamespace(type=e.EV_KEY, code=e.KEY_U, value=0),
            types.SimpleNamespace(type=e.EV_KEY, code=e.KEY_J, value=1),
            types.SimpleNamespace(type=e.EV_KEY, code=e.KEY_W, value=1),
            types.SimpleNamespace(type=e.EV_KEY, code=e.KEY_T, value=1),
            types.SimpleNamespace(type=e.EV_KEY, code=e.KEY_T, value=0),
            types.SimpleNamespace(type=e.EV_KEY, code=e.KEY_T, value=1),
            types.SimpleNamespace(type=e.EV_KEY, code=e.KEY_T, value=0),
            types.SimpleNamespace(type=e.EV_KEY, code=e.KEY_R, value=1),
            types.SimpleNamespace(type=e.EV_KEY, code=e.KEY_ESC, value=1),
        ])
        self.assertEqual(status, 0)
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
        self.assertEqual(commands.count("lift up"), 1)
        self.assertEqual(commands.count("lift down"), 1)
        lift_up = commands.index("lift up")
        lift_down = commands.index("lift down")
        # 下降只能由接下来的 J 按下触发，U 重复和松开事件均不发升降命令。
        self.assertFalse(any(
            command.startswith("lift ")
            for command in commands[lift_up + 1:lift_down]
        ))
        self.assertFalse(any(
            command.startswith("manual vel ") and command != "manual vel 0 0 0"
            for command in commands[dropped_stop + 1:rearm]
        ))

    def test_digit_presets_adjust_open_loop_duty(self):
        ecodes = self._base_ecodes(KEY_1=2, KEY_5=6, KEY_0=11)
        status, commands = self._run_session(ecodes, lambda e: [
            types.SimpleNamespace(type=e.EV_KEY, code=e.KEY_W, value=1),
            types.SimpleNamespace(type=e.EV_KEY, code=e.KEY_1, value=1),
            types.SimpleNamespace(type=e.EV_KEY, code=e.KEY_5, value=1),
            types.SimpleNamespace(type=e.EV_KEY, code=e.KEY_0, value=1),
            types.SimpleNamespace(type=e.EV_KEY, code=e.KEY_W, value=0),
            types.SimpleNamespace(type=e.EV_KEY, code=e.KEY_ESC, value=1),
        ])
        self.assertEqual(status, 0)
        self.assertIn("manual duty 0.4000", commands)
        self.assertIn("manual duty 0.4900", commands)
        self.assertIn("manual duty 0.6000", commands)
        self.assertIn("manual vel 1.0000 0.0000 0.0000", commands)


if __name__ == "__main__":
    unittest.main()
