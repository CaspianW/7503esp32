#!/usr/bin/env python3
"""ROS 2 bridge for the ESP32 ASCII console, compatible with NAV ec_adapter.

NAV's ec_bridge.launch.py remaps the five raw topics into /ec/....
The /ec/arm, /ec/stop and /ec/estop services have stable absolute names.
No automatic rearm, nonzero keepalive, lift arrival or odom TF is generated.
"""

import json
import math
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String, UInt8
from std_srvs.srv import SetBool, Trigger

from serial_bridge_protocol import Telemetry

MAX_LINE = 1024
LIFT_TEXTS = {text: 'lift ' + text for text in ('up', 'down', 'stop')}
LIFT_TEXTS.update({'lift ' + text: 'lift ' + text for text in ('up', 'down', 'stop')})


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        defaults = {
            'port': '/dev/ttyUSB0', 'baud': 115200,
            'frame_odom': 'odom', 'frame_base': 'base_link',
            'frame_imu': 'base_link',
            'cmd_timeout_s': .25, 'telemetry_timeout_s': .5,
            'max_linear_speed': .5, 'max_angular_speed': 2.,
            'arm_on_start': False, 'keepalive_period_s': 0.,
            'publish_odom': True, 'publish_imu': True,
            'publish_lift_state': True, 'publish_status': True,
            'reconnect_period_s': 2., 'startup_delay_s': 2.,
            'command_ack_timeout_s': 1.,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value, ParameterDescriptor(read_only=True))
        self.settings = {key: self.get_parameter(key).value for key in defaults}
        for key in ('cmd_timeout_s', 'telemetry_timeout_s', 'max_linear_speed',
                    'max_angular_speed', 'reconnect_period_s', 'command_ack_timeout_s'):
            value = float(self.settings[key])
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be finite and positive')
            self.settings[key] = value
        for key in ('startup_delay_s', 'keepalive_period_s'):
            value = float(self.settings[key])
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{key} must be finite and nonnegative')
            self.settings[key] = value
        if self.settings['cmd_timeout_s'] >= .5:
            raise ValueError('cmd_timeout_s must precede the 500 ms firmware watchdog')
        period = self.settings['keepalive_period_s']
        if period and period >= .5:
            raise ValueError('keepalive_period_s must be zero or below 500 ms')
        if not str(self.settings['port']).strip() or self.settings['baud'] <= 0:
            raise ValueError('port and baud are required')
        for key in ('frame_odom', 'frame_base', 'frame_imu'):
            if not str(self.settings[key]).strip():
                raise ValueError(f'{key} must not be empty')

        self._port_lock = threading.RLock()
        self._condition = threading.Condition(threading.RLock())
        self._stop_event = threading.Event()
        self._serial = None
        self._reader = None
        self._telemetry = Telemetry()
        self._session_ready = False
        self._sync_sequence = None
        self._sync_errors = 0
        self._sync_failed = None
        self._authorized = False
        self._autoarm_attempted = False
        self._last_connect_attempt = -math.inf
        self._startup_deadline = math.inf
        self._last_cmd = None
        self._zero_sent = True
        self._last_cmd_text = None
        self._rx_lines = self._rx_errors = self._tx_lines = self._write_errors = 0

        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
        self.lift_pub = self.create_publisher(UInt8, '/lift_state', 10)
        self.status_pub = self.create_publisher(String, '/serial_bridge/status', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(String, '/lift_cmd', self.on_lift_cmd, 10)
        self.create_subscription(Bool, '/estop', self.on_estop, 10)
        self.create_service(SetBool, '/ec/arm', self.on_arm)
        self.create_service(Trigger, '/ec/stop', self.on_stop)
        self.create_service(SetBool, '/ec/estop', self.on_estop_service)
        self.create_timer(.01, self.on_timer)
        self.create_timer(1., self.publish_status)
        self.get_logger().warning(
            'Explicit /ec/arm is required; reconnects never restore motion. '
            'Lift feedback is a target, not arrival. IMU mounting must be calibrated.')

    def _invalidate(self):
        # Caller holds _condition. Never carry commands across resets/reconnects.
        self._session_ready = False
        self._sync_sequence = None
        self._sync_failed = None
        self._authorized = False
        self._last_cmd = None
        self._zero_sent = True

    def connect(self):
        with self._port_lock:
            if self._serial is not None:
                return True
            if self._stop_event.is_set() or (self._reader and self._reader.is_alive()):
                return False
            now = time.monotonic()
            if now - self._last_connect_attempt < self.settings['reconnect_period_s']:
                return False
            self._last_connect_attempt = now
            try:
                port = serial.Serial(
                    str(self.settings['port']), int(self.settings['baud']),
                    timeout=.1, write_timeout=.2, exclusive=True)
            except (OSError, ValueError, serial.SerialException) as error:
                self.get_logger().warning(f'Cannot open serial port: {error}',
                                          throttle_duration_sec=10.)
                return False
            with self._condition:
                self._telemetry.reset()
                self._invalidate()
                self._startup_deadline = now + self.settings['startup_delay_s']
            self._serial = port
            self._reader = threading.Thread(target=self.read_loop, args=(port,),
                                            name='serial-reader', daemon=True)
            self._reader.start()
        return True

    def drop_port(self, expected=None):
        with self._port_lock:
            if expected is not None and self._serial is not expected:
                return  # An old reader must not close a replacement connection.
            port, self._serial = self._serial, None
            if port is not None:
                try:
                    port.close()
                except (OSError, serial.SerialException):
                    pass
            with self._condition:
                self._telemetry.reset()
                self._invalidate()
                self._condition.notify_all()

    def write_line(self, text):
        if not text or any(ch in text for ch in '\r\n\x00'):
            return False
        try:
            payload = (text + '\n').encode('ascii')
        except UnicodeEncodeError:
            return False
        if len(payload) > 96:
            return False
        with self._port_lock:
            port = self._serial
            if port is None:
                return False
            try:
                # A short write can leave a command prefix on the MCU. Drop the
                # connection rather than append another command to that prefix.
                if port.write(payload) != len(payload):
                    raise serial.SerialException('Short command write')
                self._tx_lines += 1
                self._last_cmd_text = text
                return True
            except (OSError, serial.SerialException) as error:
                self._write_errors += 1
                self.get_logger().error(f'Serial write failed: {error}')
                self.drop_port(port)
                return False

    def read_loop(self, port):
        buffer = bytearray()
        discard = False
        try:
            while not self._stop_event.is_set():
                chunk = port.read(256)
                for byte in chunk:
                    if byte == 10:
                        if not discard:
                            try:
                                self.handle_line(buffer.decode('ascii').rstrip('\r'))
                            except UnicodeDecodeError:
                                self._rx_errors += 1
                        buffer.clear()
                        discard = False
                    elif not discard:
                        buffer.append(byte)
                        if len(buffer) > MAX_LINE:
                            buffer.clear()
                            discard = True
                            self._rx_errors += 1
        except (OSError, serial.SerialException, TypeError) as error:
            if not self._stop_event.is_set():
                self.get_logger().warning(f'Serial reader stopped: {error}')
        finally:
            self.drop_port(port)

    def handle_line(self, line):
        now = time.monotonic()
        with self._condition:
            generation = self._telemetry.generation
            try:
                tag = self._telemetry.feed(line, now, self.get_clock().now().nanoseconds)
            except ValueError:
                self._rx_errors += 1
                return
            if tag not in (None, 'OK', 'ERR'):
                self._rx_lines += 1
            if generation != self._telemetry.generation:
                self._invalidate()
                self._startup_deadline = now + self.settings['startup_delay_s']
            state = self._telemetry.state
            if tag == 'S' and (not state.get('armed') or state.get('fault') or
                               state.get('emergency_stopped') or state.get('manual_mode')):
                self._authorized = False
                self._last_cmd = None
                self._zero_sent = True
            self._condition.notify_all()
        if tag == 'ERR':
            self.get_logger().warning(f'Firmware rejected a command: {line}',
                                      throttle_duration_sec=5.)

    def _motion_ready(self, now):
        return self._session_ready and self._telemetry.motion_ready(
            now, self.settings['telemetry_timeout_s'])

    def request_state(self, text, predicate, require_ready=False):
        # Only ROS callbacks write commands; the reader continues receiving S
        # records while this bounded service waits. Do not hold _condition over
        # serial I/O, so a failed write cannot deadlock the reader.
        with self._condition:
            if require_ready and not self._motion_ready(time.monotonic()):
                return False, 'Connection, fresh odometry/IMU and healthy closed-loop state required'
            self._authorized = False
            self._last_cmd = None
            self._zero_sent = True
            sequence = self._telemetry.status_sequence
            generation = self._telemetry.generation
            errors = self._telemetry.error_sequence
        if not self.write_line(text) or not self.write_line('status'):
            return False, 'Serial command could not be sent'
        deadline = time.monotonic() + self.settings['command_ack_timeout_s']
        with self._condition:
            while not self._stop_event.is_set():
                if self._telemetry.generation != generation:
                    return False, 'Connection or board reset; explicit arm required'
                if self._telemetry.error_sequence > errors:
                    return False, 'Firmware rejected the command'
                if self._telemetry.status_sequence > sequence and predicate(self._telemetry.state):
                    return True, 'Confirmed by fresh firmware status'
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False, 'Firmware state was not confirmed before timeout'
                self._condition.wait(remaining)
        return False, 'Bridge is shutting down'

    def on_arm(self, request, response):
        text = 'arm' if request.data else 'disarm'
        if request.data:
            predicate = lambda s: (s.get('armed') == 1 and s.get('fault') == 0
                                   and not s.get('emergency_stopped') and not s.get('manual_mode'))
        else:
            predicate = lambda s: s.get('armed') == 0
        response.success, response.message = self.request_state(
            text, predicate, require_ready=request.data)
        if response.success and request.data:
            with self._condition:
                self._authorized = self._motion_ready(time.monotonic()) and \
                    self._telemetry.state.get('armed') == 1
                response.success = bool(self._authorized)
                if not response.success:
                    response.message = 'Telemetry became unavailable; arm not authorized'
        return response

    def on_stop(self, request, response):
        response.success, response.message = self.request_state(
            'disarm', lambda s: s.get('armed') == 0)
        return response

    def on_estop_service(self, request, response):
        active = bool(request.data)
        response.success, response.message = self.request_state(
            'estop ' + str(int(active)),
            lambda s: s.get('emergency_stopped') == int(active) and s.get('armed') == 0)
        return response

    def on_estop(self, message):
        # Topic requests have no response; the service provides confirmation.
        with self._condition:
            self._authorized = False
            self._last_cmd = None
            self._zero_sent = True
        self.write_line('estop ' + str(int(message.data)))

    def on_cmd_vel(self, message):
        values = (message.linear.x, message.linear.y, message.angular.z)
        if not all(math.isfinite(value) for value in values):
            with self._condition:
                self._authorized = False
                self._last_cmd = None
                self._zero_sent = True
            self.write_line('disarm')
            return
        now = time.monotonic()
        with self._condition:
            allowed = (self._authorized and self._motion_ready(now)
                       and self._telemetry.state.get('armed') == 1)
        if not allowed:
            return
        linear = math.hypot(values[0], values[1])
        scale = min(1., self.settings['max_linear_speed'] / linear) if linear else 1.
        angular = max(-self.settings['max_angular_speed'],
                      min(self.settings['max_angular_speed'], values[2]))
        vx, vy = values[0] * scale, values[1] * scale
        if self.write_line(f'vel {vx:.5f} {vy:.5f} {angular:.5f}'):
            with self._condition:
                self._last_cmd = now
                self._zero_sent = not any((vx, vy, angular))

    def on_lift_cmd(self, message):
        text = LIFT_TEXTS.get(message.data.strip().lower())
        if text is None:
            self.get_logger().warning('Expected lift up/down/stop')
            return
        with self._condition:
            ready = self._session_ready and self._telemetry.status_fresh(
                time.monotonic(), self.settings['telemetry_timeout_s'])
            healthy = self._telemetry.state.get('initialized') and \
                self._telemetry.state.get('fault') == 0
        if ready and healthy:
            self.write_line(text)

    def command_zero(self):
        if self.write_line('vel 0 0 0'):
            with self._condition:
                self._last_cmd = time.monotonic()
                self._zero_sent = True

    def on_timer(self):
        if not self.connect():
            return
        now = time.monotonic()
        with self._condition:
            status_fresh = self._telemetry.status_fresh(
                now, self.settings['telemetry_timeout_s'])
            sync = self._sync_sequence
            start_sync = status_fresh and sync is None and now >= self._startup_deadline
            if start_sync:
                self._sync_sequence = self._telemetry.status_sequence
                self._sync_errors = self._telemetry.error_sequence
            elif sync is not None and self._telemetry.error_sequence > self._sync_errors and \
                    not self._session_ready:
                self._sync_failed = 'Firmware rejected startup disarm; flash matching firmware'
            elif sync is not None and not self._sync_failed and \
                    self._telemetry.status_sequence > sync and \
                    self._telemetry.state.get('armed') == 0 and status_fresh:
                self._session_ready = True
        if start_sync:
            self.write_line('disarm')
            self.write_line('status')
        self.publish_telemetry(now)
        self.check_watchdogs(now)
        if self.settings['arm_on_start'] and not self._autoarm_attempted:
            with self._condition:
                ready = self._motion_ready(now)
            if ready:
                self._autoarm_attempted = True
                result = self.on_arm(SetBool.Request(data=True), SetBool.Response())
                if not result.success:
                    self.get_logger().error(f'Startup arm failed: {result.message}')

    def check_watchdogs(self, now):
        with self._condition:
            authorized = self._authorized
            ready = self._motion_ready(now)
            last_cmd, zero_sent = self._last_cmd, self._zero_sent
        if authorized and not ready:
            # Revoke host authority even if UART is still physically open.
            with self._condition:
                self._authorized = False
            self.write_line('disarm')
            return
        if last_cmd is not None and not zero_sent and \
                now - last_cmd >= self.settings['cmd_timeout_s']:
            self.command_zero()  # Once, then let firmware expire and disarm.
            return
        period = self.settings['keepalive_period_s']
        if period and authorized and ready and last_cmd is not None and now - last_cmd >= period:
            self.command_zero()

    def publish_telemetry(self, now):
        with self._condition:
            samples = self._telemetry.take_pending(now, self.settings['telemetry_timeout_s'])
        for kind, sample in samples.items():
            if kind == 'lift':
                if self.settings['publish_lift_state']:
                    self.lift_pub.publish(UInt8(data=int(sample.values[4])))
                continue
            if kind == 'odom':
                if not self.settings['publish_odom']:
                    continue
                x, y, yaw, vx, vy, wz = sample.values
                message = Odometry()
                message.header.frame_id = str(self.settings['frame_odom'])
                message.child_frame_id = str(self.settings['frame_base'])
                message.pose.pose.position.x, message.pose.pose.position.y = x, y
                message.pose.pose.orientation.z = math.sin(yaw / 2)
                message.pose.pose.orientation.w = math.cos(yaw / 2)
                message.twist.twist.linear.x, message.twist.twist.linear.y = vx, vy
                message.twist.twist.angular.z = wz
                message.pose.covariance = self.planar_covariance(.02, .02, .05)
                message.twist.covariance = self.planar_covariance(.01, .01, .02)
                publisher = self.odom_pub
            else:
                if not self.settings['publish_imu']:
                    continue
                ax, ay, az, gx, gy, gz, _temperature = sample.values
                message = Imu()
                message.header.frame_id = str(self.settings['frame_imu'])
                message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = gx, gy, gz
                message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = ax, ay, az
                message.orientation_covariance[0] = -1.
                # Zero means unknown; NAV ec_adapter supplies its configured noise model.
                publisher = self.imu_pub
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(sample.stamp_ns, 10**9)
            publisher.publish(message)

    @staticmethod
    def planar_covariance(x, y, yaw):
        values = [0.] * 36
        for index, value in zip((0, 7, 14, 21, 28, 35), (x, y, 1e6, 1e6, 1e6, yaw)):
            values[index] = value
        return values

    def publish_status(self):
        if not self.settings['publish_status']:
            return
        now = time.monotonic()
        with self._condition:
            report = {
                'port': str(self.settings['port']), 'connected': self._serial is not None,
                'session_ready': self._session_ready, 'motion_authorized': self._authorized,
                'connection_error': self._sync_failed,
                'telemetry_fresh': self._telemetry.status_fresh(now, self.settings['telemetry_timeout_s']),
                'sensors_fresh': {kind: self._telemetry.fresh(kind, now, self.settings['telemetry_timeout_s'])
                                  for kind in ('odom', 'imu')},
                'firmware_state': dict(self._telemetry.state),
                'last_command': self._last_cmd_text,
                'rx_lines': self._rx_lines, 'rx_errors': self._rx_errors,
                'tx_lines': self._tx_lines, 'write_errors': self._write_errors,
                'lift_state_source': 'commanded_target_only', 'physical_acceptance': False,
            }
        self.status_pub.publish(String(data=json.dumps(report)))

    def shutdown(self):
        self._stop_event.set()
        # disarm preserves lift holding; the legacy stop command detaches its PWM.
        self.write_line('disarm')
        self.drop_port()
        if self._reader and self._reader is not threading.current_thread():
            self._reader.join(timeout=.5)


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
