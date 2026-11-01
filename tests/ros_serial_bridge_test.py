#!/usr/bin/env python3
"""Regression checks for the firmware/NAV wire boundary; no ROS installation needed.

Real bridge and telemetry code run against serial and ROS message/publisher
stand-ins. These checks do not establish DDS discovery or physical acceptance.
"""
import copy
import importlib.util
import math
import os
from pathlib import Path
import sys
import subprocess
import threading
import time
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from serial_bridge_protocol import Telemetry

STATUS = 'S {ms} 1 {armed} 0 {estop} {fault} 1 {imu} 1 0 1 1 0'


def status(ms=1000, armed=0, estop=0, fault=0, imu=1):
    return STATUS.format(ms=ms, armed=armed, estop=estop, fault=fault, imu=imu)


class TelemetryChecks(unittest.TestCase):
    def setUp(self):
        self.data = Telemetry()
        self.data.feed(status(), 10., 100_000_000_000)

    def sensor(self, tag='I', source=990, now=10.):
        text = f'I {source} 0 0 9.8 0 0 .1 25' if tag == 'I' else f'O {source} 1 2 .3 .1 .2 .3'
        self.data.feed(text, now, 100_000_000_000)

    def test_status_matches_firmware_field_order(self):
        self.data.feed(status(ms=123456, armed=0, fault=7, estop=1, imu=0), 10., 100_000_000_000)
        self.assertEqual(self.data.board_ms, 123456)
        self.assertEqual(self.data.state['initialized'], 1)
        self.assertEqual(self.data.state['armed'], 0)
        self.assertEqual(self.data.state['emergency_stopped'], 1)
        self.assertEqual(self.data.state['fault'], 7)
        self.assertEqual(self.data.state['encoder_ready'], 1)
        self.assertEqual(self.data.state['imu_ready'], 0)
        self.assertEqual(self.data.state['manual_mode'], 0)

    def test_new_sample_published_once_with_acquisition_stamp(self):
        self.sensor()
        first = self.data.take_pending(10., .5)['imu']
        self.assertEqual(first.stamp_ns, 99_990_000_000)
        self.assertEqual(self.data.take_pending(10.01, .5), {})
        self.sensor(now=10.02)  # Same MCU sample retransmitted by console.
        self.assertEqual(self.data.take_pending(10.02, .5), {})

    def test_other_telemetry_does_not_refresh_imu(self):
        self.sensor()
        self.data.feed(status(ms=1700), 10.7, 100_700_000_000)
        self.sensor('O', source=1690, now=10.7)
        self.assertFalse(self.data.fresh('imu', 10.7, .5))
        self.assertTrue(self.data.fresh('odom', 10.7, .5))

    def test_imu_readiness_and_zero_timestamps_are_respected(self):
        self.sensor(source=0)
        self.assertFalse(self.data.fresh('imu', 10., .5))
        self.sensor()
        self.data.feed(status(imu=0), 10., 100_000_000_000)
        self.assertNotIn('imu', self.data.take_pending(10., .5))
        self.assertFalse(self.data.motion_ready(10., .5))

    def test_status_and_sample_must_both_be_fresh(self):
        self.sensor()
        self.sensor('O')
        self.assertTrue(self.data.motion_ready(10., .5))
        self.assertFalse(self.data.motion_ready(10.51, .5))

    def test_reboot_discards_cached_measurements(self):
        self.sensor()
        generation = self.data.generation
        self.data.feed(status(ms=100), 11., 101_000_000_000)
        self.assertGreater(self.data.generation, generation)
        self.assertEqual(self.data.samples, {})
        self.assertFalse(self.data.motion_ready(11., .5))

    def test_millis_rollover_is_not_a_reboot(self):
        data = Telemetry()
        data.feed(status(ms=2**32-20), 10., 100_000_000_000)
        generation = data.generation
        data.feed(status(ms=20), 10.04, 100_040_000_000)
        data.feed(f'I {2**32-10} 0 0 9.8 0 0 .1 25', 10.04, 100_040_000_000)
        self.assertEqual(data.generation, generation)
        self.assertTrue(data.fresh('imu', 10.04, .5))
        self.assertEqual(data.samples['imu'].stamp_ns, 100_010_000_000)

    def test_malformed_records_do_not_refresh_status(self):
        for line in ('S 1 1', status().replace('1000', 'nan'),
                     status().replace('1000', '-1'), status().replace('1000', '1.5'),
                     'I 1000 0 0 inf 0 0 0 25', 'L 1 1 1 1 3 500'):
            with self.subTest(line=line), self.assertRaises(ValueError):
                self.data.feed(line, 20., 110_000_000_000)
        self.assertFalse(self.data.status_fresh(20., .5))

    def test_lift_feedback_is_only_target(self):
        self.data.feed('L 1 1 1 1 1 500', 10., 100_000_000_000)
        sample = self.data.take_pending(10., .5)['lift']
        self.assertEqual(sample.values[4], 1)
        self.assertFalse(hasattr(sample, 'arrived'))

    @unittest.skipUnless(os.environ.get('ESP32_TELEMETRY_FIXTURE'),
                         'Use tests/run_communication_tests.py for the C++ wire fixture')
    def test_actual_firmware_output_is_navigation_ready(self):
        block = subprocess.check_output([os.environ['ESP32_TELEMETRY_FIXTURE']], text=True)
        data = Telemetry()
        for line in block.splitlines():
            data.feed(line, 10., 100_000_000_000)
        self.assertEqual(data.state['initialized'], 1)
        self.assertEqual(data.state['armed'], 0)
        self.assertEqual(data.state['fault'], 0)
        self.assertTrue(data.motion_ready(10., .5))
        self.assertEqual(set(data.take_pending(10., .5)), {'odom', 'imu', 'lift'})


def vector():
    return types.SimpleNamespace(x=0., y=0., z=0., w=0.)


class Message:
    def __init__(self, **kwargs):
        self.data = kwargs.get('data', False)
        self.header = types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=0, nanosec=0), frame_id='')
        self.child_frame_id = ''
        self.pose = types.SimpleNamespace(pose=types.SimpleNamespace(
            position=vector(), orientation=vector()), covariance=[0.] * 36)
        self.twist = types.SimpleNamespace(twist=types.SimpleNamespace(
            linear=vector(), angular=vector()), covariance=[0.] * 36)
        self.linear, self.angular = vector(), vector()
        self.angular_velocity, self.linear_acceleration = vector(), vector()
        self.orientation_covariance = [0.] * 9
        self.angular_velocity_covariance = [0.] * 9
        self.linear_acceleration_covariance = [0.] * 9


class Service:
    class Request:
        def __init__(self, data=False):
            self.data = data

    class Response:
        success = False
        message = ''


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


class FakeNode:
    def __init__(self, name):
        self.params, self.services = {}, {}
        self.subscriptions = {}
        self.publishers = {}

    def declare_parameter(self, key, value, *args):
        self.params[key] = value

    def get_parameter(self, key):
        return types.SimpleNamespace(value=self.params[key])

    def create_publisher(self, message, topic, qos):
        publisher = self.publishers[topic] = Publisher()
        return publisher

    def create_subscription(self, message, topic, callback, qos):
        self.subscriptions[topic] = callback

    def create_service(self, service, name, callback):
        self.services[name] = callback

    def create_timer(self, *args):
        pass

    def get_logger(self):
        return types.SimpleNamespace(**{
            name: lambda *args, **kwargs: None for name in ('warning', 'error', 'info')})

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=100_000_000_000))


def load_bridge():
    modules = {
        'rclpy': types.ModuleType('rclpy'),
        'serial': types.SimpleNamespace(SerialException=OSError),
        'rclpy.node': types.SimpleNamespace(Node=FakeNode),
        'rclpy.qos': types.SimpleNamespace(qos_profile_sensor_data=object()),
        'rcl_interfaces.msg': types.SimpleNamespace(ParameterDescriptor=lambda **kwargs: kwargs),
        'geometry_msgs.msg': types.SimpleNamespace(Twist=Message),
        'nav_msgs.msg': types.SimpleNamespace(Odometry=Message),
        'sensor_msgs.msg': types.SimpleNamespace(Imu=Message),
        'std_msgs.msg': types.SimpleNamespace(Bool=Message, String=Message, UInt8=Message),
        'std_srvs.srv': types.SimpleNamespace(SetBool=Service, Trigger=Service),
    }
    spec = importlib.util.spec_from_file_location('bridge_under_test', ROOT/'tools/ros_serial_bridge.py')
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


bridge = load_bridge()


class Port:
    def __init__(self, node):
        self.node, self.commands = node, []
        self.armed, self.estop = 0, 0
        self.reject_arm = False
        self.reply = True
        self.closed = False

    def write(self, payload):
        text = payload.decode('ascii').strip()
        self.commands.append(text)
        if text == 'arm':
            if self.reject_arm:
                self.node.handle_line('ERR command_or_state')
            else:
                self.armed = 1
        elif text == 'disarm':
            self.armed = 0
        elif text.startswith('estop '):
            self.estop, self.armed = int(text[-1]), 0
        if text == 'status' and self.reply:
            self.node.handle_line(status(armed=self.armed, estop=self.estop))
        return len(payload)

    def close(self):
        self.closed = True


class BridgeChecks(unittest.TestCase):
    def setUp(self):
        self.node = bridge.SerialBridge()
        self.port = Port(self.node)
        self.node._serial = self.port
        self.node._session_ready = True
        self.node.handle_line(status())
        self.node.handle_line('O 990 1 2 .3 .1 .2 .3')
        self.node.handle_line('I 990 0 0 9.8 0 0 .1 25')
        self.node.settings['command_ack_timeout_s'] = .01

    def arm(self):
        response = self.node.on_arm(Service.Request(True), Service.Response())
        self.assertTrue(response.success, response.message)

    def move(self):
        message = Message()
        message.linear.x, message.linear.y, message.angular.z = .6, .8, 3.
        self.node.on_cmd_vel(message)

    def test_default_does_not_arm_or_forward_motion(self):
        self.assertFalse(self.node.settings['arm_on_start'])
        self.move()
        self.assertEqual(self.port.commands, [])
        self.assertIn('/ec/arm', self.node.services)
        self.assertIn('/estop', self.node.subscriptions)

    def test_startup_synchronizes_disarm_before_accepting_arm(self):
        self.node._session_ready = False
        self.node._startup_deadline = time.monotonic() + 10.
        self.node.on_timer()
        self.assertEqual(self.port.commands, [])
        self.assertFalse(self.node._session_ready)
        self.node._startup_deadline = 0.
        self.node.on_timer()
        self.assertEqual(self.port.commands, ['disarm', 'status'])
        self.assertFalse(self.node._session_ready)
        self.node.on_timer()
        self.assertTrue(self.node._session_ready)
        self.assertFalse(self.node._authorized)
        self.arm()

    def test_legacy_firmware_rejecting_disarm_cannot_complete_startup(self):
        self.node._session_ready = False
        self.node._startup_deadline = 0.
        original = self.port.write

        def legacy_write(payload):
            if payload == b'disarm\n':
                self.node.handle_line('ERR command_or_state')
                return len(payload)
            return original(payload)

        with patch.object(self.port, 'write', side_effect=legacy_write):
            self.node.on_timer()
            self.node.on_timer()
        self.assertFalse(self.node._session_ready)
        self.assertIsNotNone(self.node._sync_failed)
        result = self.node.on_arm(Service.Request(True), Service.Response())
        self.assertFalse(result.success)

    def test_reader_discards_oversized_and_non_ascii_records(self):
        node = self.node

        class ChunkPort:
            def __init__(self):
                self.chunks = iter([
                    b'X' * 1025 + b'\n' + status().encode()[:15],
                    status().encode()[15:] + b'\n\xff\n',
                    b'I 1000 0 0 9.8 0 0 .1 25\n',
                ])

            def read(self, size):
                try:
                    return next(self.chunks)
                except StopIteration:
                    node._stop_event.set()
                    return b''

            def close(self):
                pass

        port = ChunkPort()
        node._serial = port
        received = []
        original = node.handle_line

        def capture(line):
            received.append(line)
            original(line)

        with patch.object(node, 'handle_line', side_effect=capture):
            node.read_loop(port)
        self.assertEqual(received, [status(), 'I 1000 0 0 9.8 0 0 .1 25'])
        self.assertEqual(node._rx_errors, 2)

    def test_service_wait_is_satisfied_by_reader_thread(self):
        self.port.reply = False
        self.node.settings['command_ack_timeout_s'] = 1.
        waiting = threading.Event()

        def receive():
            waiting.wait(timeout=1.)
            self.node.handle_line(status(armed=1))

        original = self.port.write

        def send(payload):
            result = original(payload)
            if payload == b'status\n':
                waiting.set()
            return result

        reader = threading.Thread(target=receive)
        reader.start()
        try:
            with patch.object(self.port, 'write', side_effect=send):
                self.arm()
        finally:
            reader.join(timeout=1.)
        self.assertFalse(reader.is_alive())

    def test_confirmed_arm_then_clamped_velocity(self):
        self.arm()
        self.move()
        self.assertEqual(self.port.commands, ['arm', 'status', 'vel 0.30000 0.40000 2.00000'])

    def test_firmware_rejection_is_not_reported_as_success(self):
        self.port.reject_arm = True
        result = self.node.on_arm(Service.Request(True), Service.Response())
        self.assertFalse(result.success)
        self.assertFalse(self.node._authorized)
        self.move()
        self.assertFalse(any(c.startswith('vel') for c in self.port.commands))

    def test_rejected_disarm_is_not_success_even_if_already_stopped(self):
        original = self.port.write

        def reject_disarm(payload):
            if payload == b'disarm\n':
                self.node.handle_line('ERR command_or_state')
                return len(payload)
            return original(payload)

        with patch.object(self.port, 'write', side_effect=reject_disarm):
            result = self.node.on_stop(Service.Request(), Service.Response())
        self.assertFalse(result.success)

    def test_service_requires_a_new_status_confirmation(self):
        self.port.reply = False
        result = self.node.on_arm(Service.Request(True), Service.Response())
        self.assertFalse(result.success)
        self.assertFalse(self.node._authorized)

    def test_disarm_and_shutdown_preserve_lift_wire_semantics(self):
        self.arm()
        result = self.node.on_stop(Service.Request(), Service.Response())
        self.assertTrue(result.success)
        self.assertIn('disarm', self.port.commands)
        self.assertNotIn('stop', self.port.commands)
        self.node.shutdown()
        self.assertEqual(self.port.commands[-1], 'disarm')
        self.assertTrue(self.port.closed)

    def test_estop_release_still_requires_explicit_arm(self):
        self.arm()
        for active in (True, False):
            result = self.node.on_estop_service(Service.Request(active), Service.Response())
            self.assertTrue(result.success)
        self.move()
        self.assertFalse(any(c.startswith('vel') for c in self.port.commands))
        self.arm()
        self.move()
        self.assertTrue(self.port.commands[-1].startswith('vel'))

    def test_firmware_timeout_requires_explicit_rearm(self):
        self.arm()
        self.move()
        self.port.armed = 0
        self.node.handle_line(status(armed=0))
        count = len(self.port.commands)
        self.move()
        self.assertEqual(len(self.port.commands), count)
        self.arm()
        self.move()
        self.assertTrue(self.port.commands[-1].startswith('vel'))

    def test_connection_drop_clears_samples_and_motion_authority(self):
        self.arm()
        self.node._autoarm_attempted = True
        self.move()
        self.node.drop_port(self.port)
        self.assertFalse(self.node._authorized)
        self.assertIsNone(self.node._last_cmd)
        self.assertEqual(self.node._telemetry.samples, {})
        self.assertTrue(self.node._autoarm_attempted)
        replacement = Port(self.node)
        self.node._serial = replacement
        self.node.drop_port(self.port)  # Late exit from an old reader.
        self.assertIs(self.node._serial, replacement)
        self.assertFalse(replacement.closed)

    def test_reset_revokes_motion_without_replaying_commands(self):
        self.arm()
        self.move()
        self.node.handle_line(status(ms=100))
        self.assertFalse(self.node._authorized)
        self.assertFalse(self.node._session_ready)
        self.assertIsNone(self.node._last_cmd)

    def test_watchdog_sends_one_zero_then_silence(self):
        self.arm()
        self.move()
        now = self.node._last_cmd
        self.node.check_watchdogs(now + .26)
        self.node.check_watchdogs(now + .27)
        self.assertEqual(self.port.commands.count('vel 0 0 0'), 1)
        self.assertNotIn('stop', self.port.commands)

    def test_stale_sensor_disarms_even_with_live_serial_port(self):
        self.arm()
        self.move()
        self.node.check_watchdogs(time.monotonic() + .6)
        self.assertEqual(self.port.commands[-1], 'disarm')
        self.assertFalse(self.node._authorized)

    def test_publishing_does_not_repeat_or_refresh_samples(self):
        self.node.publish_telemetry(time.monotonic())
        self.node.publish_telemetry(time.monotonic())
        for pub in (self.node.odom_pub, self.node.imu_pub):
            self.assertEqual(len(pub.messages), 1)
            stamp = pub.messages[0].header.stamp
            self.assertEqual((stamp.sec, stamp.nanosec), (99, 990_000_000))
        self.assertEqual(self.node.odom_pub.messages[0].header.frame_id, 'odom')
        self.assertEqual(self.node.imu_pub.messages[0].orientation_covariance[0], -1.)

    def test_short_write_invalidates_connection(self):
        self.arm()
        with patch.object(self.port, 'write', return_value=1):
            self.assertFalse(self.node.write_line('vel 0 0 0'))
        self.assertFalse(self.node._authorized)
        self.assertIsNone(self.node._serial)

    def test_nonfinite_velocity_disarms(self):
        self.arm()
        message = Message()
        message.linear.x = math.nan
        self.node.on_cmd_vel(message)
        self.assertEqual(self.port.commands[-1], 'disarm')
        self.assertFalse(self.node._authorized)

    def test_wire_injection_is_rejected(self):
        for text in ('arm\nvel 1 0 0', 'arm\x00', 'a' * 96, '升降'):
            self.assertFalse(self.node.write_line(text))
        self.assertEqual(self.port.commands, [])

    def test_lift_text_accepts_nav_full_uart_form(self):
        self.node.on_lift_cmd(Message(data='lift up'))
        self.node.on_lift_cmd(Message(data='down'))
        self.node.on_lift_cmd(Message(data='lift up\narm'))
        self.assertEqual(self.port.commands, ['lift up', 'lift down'])


if __name__ == '__main__':
    unittest.main()
