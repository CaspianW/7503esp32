#!/usr/bin/env python3
"""Run communication regressions and the actual C++ -> Python wire contract."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    sources = [str(path) for directory in ('src/app', 'src/communication', 'src/control/src')
               for path in sorted((ROOT / directory).glob('*.cpp'))]
    sources += [str(ROOT / 'src/drivers/src' / name)
                for name in ('encoder.cpp', 'motor_driver.cpp', 'servo_driver.cpp')]
    sources += [str(ROOT / 'tests/fakes' / name) for name in ('hardware.cpp', 'imu.cpp')]
    with tempfile.TemporaryDirectory(prefix='esp32-communication-') as directory:
        for name in ('serial_protocol', 'serial_console', 'robot_controller_safety',
                     'lift_control', 'safety', 'serial_telemetry_fixture'):
            source = name + ('.cpp' if name.endswith('_fixture') else '_test.cpp')
            binary = str(Path(directory) / name)
            subprocess.run(['g++', '-std=c++11', '-Wall', '-Wextra', '-Werror',
                            '-I' + str(ROOT / 'tests/fakes'), str(ROOT / 'tests' / source),
                            *sources, '-o', binary], check=True)
            if not name.endswith('_fixture'):
                subprocess.run([binary], check=True)
                print(name + ': PASS', flush=True)
        subprocess.run([sys.executable, str(ROOT / 'tests/ros_serial_bridge_test.py'), '-v'],
                       env={**os.environ, 'ESP32_TELEMETRY_FIXTURE': binary}, check=True)


if __name__ == '__main__':
    main()
