#!/usr/bin/env python3
"""ROS 2 <-> ESP32 ASCII serial bridge for the DASE7503 logistics robot.

This file is the contract boundary between the EC firmware (this repository,
`src/communication/`) and the NAV stack (`dase7503-group-project`).  It is the
only script the NAV launcher expects at `tools/ros_serial_bridge.py`:

    bash scripts/nav.sh hardware start_bridge:=true \
        bridge_root:=<this-repo> port:=/dev/ttyUSB0

Topic contract (default names; the NAV launcher remaps them, unchanged, to the
`/ec/...` namespace consumed by `robot_bringup/ec_adapter`):

    publish  /wheel/odom    nav_msgs/Odometry   frame odom -> base_link
    publish  /imu/data_raw  sensor_msgs/Imu     frame base_link, no attitude
    publish  /lift_state    std_msgs/UInt8      0 = lowered, 1 = raised,
                                                2 = no active target
    subscribe /cmd_vel      geometry_msgs/Twist -> "vel vx vy wz"
    subscribe /lift_cmd     std_msgs/String     -> "lift up|down|stop"

Wire protocol is exactly the firmware's ASCII serial console
(`src/communication/serial_protocol.cpp`, `serial_console.cpp`), 115200 8N1:

    host -> board : arm | stop | estop 0|1 | status | vel <vx> <vy> <wz>
                    | lift up | lift down | lift stop
    board -> host : "OK" / "ERR ..." acknowledgment, plus a 10 Hz telemetry
                    block: S / E / O / I / L lines, one record per line.

Deliberate limits, matching what the firmware actually measures:

* No lift arrival signal exists.  `/lift_state` republishes the commanded
  target reported in the L line; it is not evidence that the mechanism arrived.
* The IMU is published in raw chip axes with zero covariances; no orientation
  is fabricated.  Physical axis sign and mounting rotation are NAV-side
  calibration decisions (`ec_20261025.yaml`), not something this file invents.
* No `arm` and no keepalive traffic is generated unless explicitly requested by
  parameter.  The firmware's own 500 ms command watchdog must stay able to
  stop the chassis.
"""

import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String, UInt8

try:
    import serial
except ImportError as error:  # pragma: no cover - dependency guard
    raise SystemExit(
        'pyserial is required: sudo apt install python3-serial') from error


# Lift command texts accepted from /lift_cmd.  The full UART form ("lift up")
# is also the firmware form, so it is forwarded verbatim; the bare verbs are
# accepted for convenience and prefixed here.
LIFT_TEXTS = {
    'lift up': 'lift up',
    'lift down': 'lift down',
    'lift stop': 'lift stop',
    'up': 'lift up',
    'down': 'lift down',
    'stop': 'lift stop',
}

# Telemetry record lengths, including the tag token itself.
RECORD_TOKENS = {'S': 14, 'E': 13, 'O': 8, 'I': 9, 'L': 7, 'P': 9}
MAX_LINE = 1024


def yaw_to_quaternion(yaw):
    """Planar yaw to a unit quaternion; roll and pitch stay exactly zero."""
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        defaults = {
            'port': '/dev/ttyUSB0',
            'baud': 115200,
            'frame_odom': 'odom',
            'frame_base': 'base_link',
            'frame_imu': 'base_link',
            'cmd_timeout_s': 0.25,
            'telemetry_timeout_s': 0.5,
            'max_linear_speed': 0.5,
            'max_angular_speed': 2.0,
            'arm_on_start': False,
            'keepalive_period_s': 0.0,
            'publish_odom': True,
            'publish_imu': True,
            'publish_lift_state': True,
            'publish_status': True,
            'reconnect_period_s': 2.0,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.settings = {key: self.get_parameter(key).value for key in defaults}

        for key in ('cmd_timeout_s', 'telemetry_timeout_s', 'max_linear_speed',
                    'max_angular_speed', 'reconnect_period_s'):
            value = float(self.settings[key])
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{key} must be finite and positive')
            self.settings[key] = value

        self._write_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._serial = None
        self._reader = None
        self._stop_event = threading.Event()

        self._latest = {}
        self._odom_ready = False
        self._imu_ready = False
        self._lift_ready = False
        # Frame counters: the timer runs far faster than the 10 Hz telemetry, so
        # each record must be forwarded once, not once per timer tick.
        self._seq = {'odom': 0, 'imu': 0, 'lift': 0}
        self._pub_seq = {'odom': 0, 'imu': 0, 'lift': 0}
        self._state = {
            'armed': None, 'motion_allowed': None, 'emergency_stopped': None,
            'initialized': None, 'fault': None, 'imu_ready': None,
        }
        self._last_rx = None
        self._last_cmd = None
        self._last_cmd_text = None
        self._zero_sent = True
        self._arm_requested = False
        self._rx_lines = 0
        self._rx_errors = 0
        self._tx_lines = 0
        self._write_errors = 0
        self._connected = False
        self._last_connect_attempt = 0.0

        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw',
                                             qos_profile_sensor_data)
        self.lift_pub = self.create_publisher(UInt8, '/lift_state', 10)
        self.status_pub = self.create_publisher(String, '/serial_bridge/status', 10)

        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(String, '/lift_cmd', self.on_lift_cmd, 10)

        self.create_timer(0.01, self.on_timer)
        self.create_timer(1.0, self.publish_status)
        self.get_logger().warning(
            f'Serial bridge on {self.settings["port"]} @ {self.settings["baud"]}: '
            'lift state is a commanded target, not an arrival sensor; '
            'IMU axes are raw chip axes with zero covariance')

    # ---------------------------------------------------------------- serial
    def connect(self):
        if self._connected:
            return True
        now = time.monotonic()
        if now - self._last_connect_attempt < self.settings['reconnect_period_s']:
            return False
        self._last_connect_attempt = now
        try:
            self._serial = serial.Serial(
                str(self.settings['port']), int(self.settings['baud']),
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.1, write_timeout=0.2)
        except (OSError, ValueError) as error:
            self._serial = None
            self.get_logger().warning(
                f'Cannot open {self.settings["port"]}: {error}',
                throttle_duration_sec=10.0)
            return False
        self._stop_event.clear()
        self._reader = threading.Thread(target=self.read_loop, name='serial-reader',
                                        daemon=True)
        self._reader.start()
        self._connected = True
        self.get_logger().info(f'Serial port opened: {self.settings["port"]}')
        return True

    def write_line(self, text):
        """Send one ASCII line.  Only this file may compose command lines."""
        if any(ch in text for ch in '\r\n\x00'):
            self.get_logger().error('Refusing embedded newline or NUL in command')
            return False
        payload = (text + '\n').encode('ascii', errors='strict')
        if len(payload) > 95:
            self.get_logger().error(f'Command too long for firmware parser: {text!r}')
            return False
        port = self._serial
        if port is None:
            return False
        try:
            with self._write_lock:
                port.write(payload)
                port.flush()
            self._tx_lines += 1
            return True
        except (OSError, serial.SerialException) as error:
            self._write_errors += 1
            self.get_logger().error(f'Serial write failed: {error}')
            self.drop_port()
            return False

    def drop_port(self):
        self._connected = False
        port, self._serial = self._serial, None
        if port is not None:
            try:
                port.close()
            except (OSError, serial.SerialException):
                pass

    def read_loop(self):
        buffer = bytearray()
        port = self._serial
        while not self._stop_event.is_set() and port is not None:
            try:
                chunk = port.read(256)
            except (OSError, serial.SerialException, TypeError):
                break
            if not chunk:
                continue
            buffer.extend(chunk)
            while True:
                index = buffer.find(b'\n')
                if index < 0:
                    break
                raw = bytes(buffer[:index])
                del buffer[:index + 1]
                if len(raw) > MAX_LINE:
                    self._rx_errors += 1
                    continue
                self.handle_line(raw.decode('ascii', errors='replace').strip('\r'))
            if len(buffer) > MAX_LINE * 4:
                buffer.clear()
        self.drop_port()

    # ------------------------------------------------------------- telemetry
    def handle_line(self, line):
        if not line:
            return
        parts = line.split()
        tag = parts[0]
        if tag in ('OK',):
            return
        if tag == 'ERR':
            self.get_logger().warning(f'Firmware rejected a command: {line}')
            return
        expected = RECORD_TOKENS.get(tag)
        if expected is None or len(parts) != expected:
            self._rx_errors += 1
            error = f'Unrecognised telemetry record: {line[:80]}'
            self.get_logger().warning(error, throttle_duration_sec=5.0)
            return
        try:
            values = [float(token) for token in parts[1:]]
        except ValueError:
            self._rx_errors += 1
            return
        if not all(math.isfinite(value) for value in values):
            self._rx_errors += 1
            return
        self._rx_lines += 1
        if tag == 'S':
            with self._state_lock:
                self._state.update({
                    'initialized': int(values[0]),
                    'armed': int(values[1]),
                    'motion_allowed': int(values[2]),
                    'emergency_stopped': int(values[3]),
                    'fault': int(values[4]),
                    'encoder_ready': int(values[5]),
                    'imu_ready': int(values[6]),
                    'speed_ready': int(values[7]),
                })
            self._last_rx = time.monotonic()
        elif tag == 'O':
            # O <ms> x y yaw vx vy wz
            self._latest['odom'] = values[1:]
            self._odom_ready = True
            self._seq['odom'] += 1
            self._last_rx = time.monotonic()
        elif tag == 'I':
            # I <ms> ax ay az gx gy gz temperature
            self._latest['imu'] = values[1:]
            self._imu_ready = True
            self._seq['imu'] += 1
            self._last_rx = time.monotonic()
        elif tag == 'L':
            # L outputs_enabled configured ready active target pulse
            self._latest['lift'] = values
            self._lift_ready = True
            self._seq['lift'] += 1
            self._last_rx = time.monotonic()

    # -------------------------------------------------------------- commands
    def on_cmd_vel(self, message):
        values = (message.linear.x, message.linear.y, message.angular.z)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('Non-finite velocity received; forcing zero')
            self.command_zero()
            return
        linear = math.hypot(values[0], values[1])
        scale = 1.0
        limit = self.settings['max_linear_speed']
        if linear > limit:
            scale = limit / linear
        angular = max(-self.settings['max_angular_speed'],
                      min(self.settings['max_angular_speed'], values[2]))
        vx, vy = values[0] * scale, values[1] * scale
        self.write_line(f'vel {vx:.5f} {vy:.5f} {angular:.5f}')
        self._last_cmd = time.monotonic()
        self._last_cmd_text = f'vel {vx:.5f} {vy:.5f} {angular:.5f}'
        self._zero_sent = vx == 0.0 and vy == 0.0 and angular == 0.0

    def on_lift_cmd(self, message):
        text = LIFT_TEXTS.get(message.data.strip().lower())
        if text is None:
            self.get_logger().warning(
                f'Ignoring /lift_cmd {message.data!r}; expected lift up/down/stop')
            return
        self.write_line(text)
        self._last_cmd_text = text

    def command_zero(self):
        self.write_line('vel 0.0 0.0 0.0')
        self._last_cmd = time.monotonic()
        self._last_cmd_text = 'vel 0.0 0.0 0.0'
        self._zero_sent = True

    # ---------------------------------------------------------------- timers
    def on_timer(self):
        if not self.connect():
            return
        now = time.monotonic()
        self.publish_telemetry(now)
        self.check_watchdogs(now)
        self.maybe_arm()

    def publish_telemetry(self, now):
        fresh = self._last_rx is not None and \
            now - self._last_rx <= self.settings['telemetry_timeout_s']
        if not fresh:
            return
        stamp = self.get_clock().now().to_msg()

        if self.settings['publish_odom'] and self._odom_ready and \
                self._seq['odom'] != self._pub_seq['odom']:
            x, y, yaw, vx, vy, wz = self._latest['odom']
            message = Odometry()
            message.header.stamp = stamp
            message.header.frame_id = str(self.settings['frame_odom'])
            message.child_frame_id = str(self.settings['frame_base'])
            message.pose.pose.position.x = x
            message.pose.pose.position.y = y
            message.pose.pose.position.z = 0.0
            (message.pose.pose.orientation.x, message.pose.pose.orientation.y,
             message.pose.pose.orientation.z, message.pose.pose.orientation.w) = \
                yaw_to_quaternion(yaw)
            message.twist.twist.linear.x = vx
            message.twist.twist.linear.y = vy
            message.twist.twist.angular.z = wz
            # Encoder dead reckoning: planar pose, unbounded yaw growth.
            message.pose.covariance = [
                0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.05]
            message.twist.covariance = [
                0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.02]
            self.odom_pub.publish(message)
            self._pub_seq['odom'] = self._seq['odom']

        if self.settings['publish_imu'] and self._imu_ready and \
                self._seq['imu'] != self._pub_seq['imu']:
            ax, ay, az, gx, gy, gz, _temperature = self._latest['imu']
            message = Imu()
            message.header.stamp = stamp
            message.header.frame_id = str(self.settings['frame_imu'])
            message.angular_velocity.x = gx
            message.angular_velocity.y = gy
            message.angular_velocity.z = gz
            message.linear_acceleration.x = ax
            message.linear_acceleration.y = ay
            message.linear_acceleration.z = az
            # Zero covariance: the firmware reports no calibrated noise model.
            message.angular_velocity_covariance = [0.0] * 9
            message.linear_acceleration_covariance = [0.0] * 9
            # No attitude is fused on the board; state that explicitly.
            message.orientation_covariance = [-1.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0]
            self.imu_pub.publish(message)
            self._pub_seq['imu'] = self._seq['imu']

        if self.settings['publish_lift_state'] and self._lift_ready and \
                self._seq['lift'] != self._pub_seq['lift']:
            _enabled, _configured, _ready, _active, target, _pulse = \
                self._latest['lift']
            value = int(target)
            self.lift_pub.publish(UInt8(data=value if value in (0, 1) else 2))
            self._pub_seq['lift'] = self._seq['lift']

    def check_watchdogs(self, now):
        if self._last_cmd is not None and not self._zero_sent and \
                now - self._last_cmd >= self.settings['cmd_timeout_s']:
            # One zero, then silence: the firmware's own 500 ms watchdog must
            # still be able to stop and disarm the chassis.
            self.command_zero()
        period = self.settings['keepalive_period_s']
        if period > 0.0 and self._last_cmd is not None and \
                now - self._last_cmd >= period:
            # Optional zero-velocity refresh keeps the board armed during
            # bench work.  It never carries a non-zero velocity.
            self.write_line('vel 0.0 0.0 0.0')
            self._last_cmd = now
            self._last_cmd_text = 'vel 0.0 0.0 0.0 (keepalive)'

    def maybe_arm(self):
        if not self.settings['arm_on_start'] or self._arm_requested:
            return
        with self._state_lock:
            armed = self._state['armed']
            fault = self._state['fault']
        if armed is None:
            return
        if armed == 0 and fault == 0:
            if self.write_line('arm'):
                self._arm_requested = True
                self.get_logger().warning(
                    'arm_on_start: "arm" sent; chassis control is now enabled')
        elif armed == 1:
            self._arm_requested = True

    def publish_status(self):
        if not self.settings['publish_status']:
            return
        now = time.monotonic()
        with self._state_lock:
            state = dict(self._state)
        report = {
            'port': str(self.settings['port']),
            'connected': self._connected,
            'telemetry_fresh': self._last_rx is not None and
            now - self._last_rx <= self.settings['telemetry_timeout_s'],
            'telemetry_age_s': None if self._last_rx is None
            else round(now - self._last_rx, 3),
            'rx_lines': self._rx_lines,
            'rx_errors': self._rx_errors,
            'telemetry_frames': dict(self._seq),
            'tx_lines': self._tx_lines,
            'write_errors': self._write_errors,
            'last_command': self._last_cmd_text,
            'lift_state_source': 'commanded_target_only',
            'firmware_state': state,
            'physical_acceptance': False,
        }
        self.status_pub.publish(String(data=json.dumps(report)))

    def shutdown(self):
        self._stop_event.set()
        if self._connected:
            # Stop the chassis and disarm.  Lift holding is left untouched:
            # a stop pulse can release servo torque.
            self.write_line('stop')
        self.drop_port()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SerialBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
