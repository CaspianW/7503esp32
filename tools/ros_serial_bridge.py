#!/usr/bin/env python3
"""@file ros_serial_bridge.py
@brief ESP32 ASCII 串口控制台与 ROS 2 Jazzy 之间的双向桥接节点。
@author Huang Boyan
@date 2026-10-25
@version 0.1.0

固件侧只提供 USB-UART ASCII 行协议（见 src/communication/serial_protocol.* 与
serial_console.*），本节点在树莓派侧把它翻译成 ROS 2 话题：

下行（ROS 2 -> 串口）
  /cmd_vel   geometry_msgs/Twist   -> "vel <vx> <vy> <wz>"（底盘轮速闭环，需先 arm）
  /lift_cmd  std_msgs/String       -> "lift up" / "lift down" / "lift stop"
  /estop     std_msgs/Bool         -> "estop 1" / "estop 0"

上行（串口 -> ROS 2）
  O 帧 -> /wheel/odom     nav_msgs/Odometry（里程计输入，供 EKF 融合）
  I 帧 -> /imu/data_raw   sensor_msgs/Imu（无姿态解算，orientation 标记为不可用）
  L 帧 -> /lift_state     std_msgs/UInt8（0=落下，1=抬起，2=运动中）

约定：
  1. 桥接节点本身不发布 odom->base_link 变换，该变换由树莓派侧 EKF 唯一发布，
     否则会出现双发布冲突。
  2. 固件 telemetryPeriod 当前为 100 ms（10 Hz），因此 /wheel/odom 与
     /imu/data_raw 的实际频率就是 10 Hz，接口表要求的高于 30/50 Hz 需固件侧调整。
  3. 固件在 500 ms 内收不到 vel 命令会自动锁停，本节点再加一层看门狗，
     超时后主动补发零速，避免链路静默时底盘保持运动。
"""

import math
import sys
import threading
import time

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Bool, String, UInt8
except ImportError as error:  # 未 source ROS 2 环境时给出明确提示，不抛出堆栈
    print("未找到 ROS 2 Python 环境，请先执行：source /opt/ros/jazzy/setup.bash",
          file=sys.stderr)
    print(f"导入失败：{error}", file=sys.stderr)
    raise SystemExit(2)

try:
    import serial
except ImportError:
    print("缺少 pyserial，请执行：sudo apt install python3-serial", file=sys.stderr)
    raise SystemExit(2)


LIFT_LOWERED = 0
LIFT_RAISED = 1
LIFT_MOVING = 2


def clamp(value, limit):
    """把不可信的外部输入限制在固件允许的速度范围内。"""
    if not math.isfinite(value):
        return 0.0
    return max(-limit, min(limit, value))


class SerialBridge(Node):
    """把 ESP32 串口控制台桥接到 ROS 2 话题。"""

    def __init__(self):
        super().__init__("esp32_serial_bridge")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("wheel_odom_topic", "/wheel/odom")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("lift_cmd_topic", "/lift_cmd")
        self.declare_parameter("lift_state_topic", "/lift_state")
        self.declare_parameter("estop_topic", "/estop")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        # 与 src/config/robot_config.h 的 maxLinearSpeed / maxAngularSpeed 保持一致
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 2.0)
        # 与固件 commandTimeout（500 ms）对齐
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("min_command_period", 0.02)
        self.declare_parameter("control_period", 0.02)
        self.declare_parameter("serial_read_timeout", 0.05)
        self.declare_parameter("arm_attempts", 3)
        self.declare_parameter("boot_delay", 2.0)

        self._port = self.get_parameter("port").value
        self._baud = int(self.get_parameter("baud").value)
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._max_linear = float(self.get_parameter("max_linear_speed").value)
        self._max_angular = float(self.get_parameter("max_angular_speed").value)
        self._command_timeout = float(self.get_parameter("command_timeout").value)
        self._min_command_period = float(self.get_parameter("min_command_period").value)
        self._control_period = float(self.get_parameter("control_period").value)
        self._read_timeout = float(self.get_parameter("serial_read_timeout").value)
        self._arm_attempts = int(self.get_parameter("arm_attempts").value)
        self._boot_delay = float(self.get_parameter("boot_delay").value)

        self._connection = None
        self._write_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._pending_velocity = None
        self._reader = None
        self._running = threading.Event()
        self._armed_event = threading.Event()
        self._armed = False
        self._estop_active = False
        self._last_send = 0.0
        self._last_drive = 0.0
        self._zero_sent = True
        self._link_error_reported = False
        self._timer = None
        self._counts = {"odom": 0, "imu": 0, "lift": 0, "ok": 0, "err": 0}

        self._odom_pub = self.create_publisher(
            Odometry, self.get_parameter("wheel_odom_topic").value, 10)
        self._imu_pub = self.create_publisher(
            Imu, self.get_parameter("imu_topic").value, 10)
        self._lift_pub = self.create_publisher(
            UInt8, self.get_parameter("lift_state_topic").value, 10)

        self.create_subscription(
            Twist, self.get_parameter("cmd_vel_topic").value, self._on_cmd_vel, 10)
        self.create_subscription(
            String, self.get_parameter("lift_cmd_topic").value, self._on_lift_cmd, 10)
        self.create_subscription(
            Bool, self.get_parameter("estop_topic").value, self._on_estop, 10)

    # ------------------------------------------------------------------ 启动

    def start(self):
        if not self._open_serial():
            return False
        self._running.set()
        self._reader = threading.Thread(
            target=self._serial_loop, name="esp32_serial_reader", daemon=True)
        self._reader.start()
        if not self._arm_controller():
            self.get_logger().error(
                "连续多次没有收到固件 arm 确认，底盘未使能；请检查串口设备、"
                "波特率与固件是否已烧录")
            return False
        self._timer = self.create_timer(self._control_period, self._on_control_timer)
        self.get_logger().info(
            f"桥接已启动：{self._port}@{self._baud}，速度上限 "
            f"{self._max_linear:.2f} m/s / {self._max_angular:.2f} rad/s")
        self.get_logger().warn(
            "固件遥测周期为 10 Hz，/wheel/odom 与 /imu/data_raw 的实际频率受其限制")
        return True

    def _open_serial(self):
        kwargs = dict(port=self._port, baudrate=self._baud,
                      timeout=self._read_timeout, write_timeout=0.2)
        try:
            # exclusive 避免与 minicom/键盘脚本同时抢占同一个串口
            self._connection = serial.Serial(exclusive=True, **kwargs)
        except (TypeError, ValueError):
            self._connection = serial.Serial(**kwargs)
        except (serial.SerialException, OSError) as error:
            self.get_logger().error(f"无法打开串口 {self._port}：{error}")
            return False
        return True

    def _arm_controller(self):
        # 打开串口会触发 DTR 复位，等固件启动完成后再发命令
        time.sleep(self._boot_delay)
        self._reset_input()
        for attempt in range(1, self._arm_attempts + 1):
            if self._send("arm") and self._armed_event.wait(1.5):
                self._armed = True
                self._last_drive = time.monotonic()
                self._zero_sent = True
                self.get_logger().info("底盘已使能（状态帧 armed=1）")
                return True
            self.get_logger().warn(f"第 {attempt} 次 arm 未获确认，准备重试")
            time.sleep(0.2)
            self._reset_input()
        return False

    def _reset_input(self):
        connection = self._connection
        if connection is None:
            return
        try:
            connection.reset_input_buffer()
        except (serial.SerialException, OSError):
            pass

    # ------------------------------------------------------------------ 串口

    def _serial_loop(self):
        while self._running.is_set() and rclpy.ok():
            connection = self._connection
            if connection is None:
                break
            try:
                raw = connection.readline()
            except (serial.SerialException, OSError) as error:
                if self._running.is_set():
                    self.get_logger().error(f"串口读取失败，桥接停止：{error}")
                break
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if line:
                self._handle_line(line)

    def _handle_line(self, line):
        fields = line.split()
        if not fields:
            return
        tag = fields[0]
        try:
            if tag == "OK":
                self._counts["ok"] += 1
            elif tag == "ERR":
                self._counts["err"] += 1
                self.get_logger().warn(f"固件拒绝命令：{line}")
            elif tag == "S":
                self._on_status(fields)
            elif tag == "O":
                self._on_odometry(fields)
            elif tag == "I":
                self._on_imu(fields)
            elif tag == "L":
                self._on_lift(fields)
            # E 行（轮速目标/实测/占空比）与 P 行（台架 PWM 回读）当前不上行到 ROS 2
        except (IndexError, ValueError) as error:
            self.get_logger().debug(f"无法解析遥测行 {line!r}：{error}")

    def _on_status(self, fields):
        # S <millis> <initialized> <armed> <motionAllowed> <estop> <fault>
        #   <encoderReady> <imuReady> <speedReady> <bench> <motorOut>
        #   <chassisCtl> <manual>
        if len(fields) < 14:
            return
        armed = fields[3] == "1"
        if armed and not self._armed_event.is_set():
            self._armed_event.set()
        if fields[5] == "1" and not self._estop_active:
            self._estop_active = True
            self.get_logger().warn("固件报告急停处于激活状态")
        elif fields[5] == "0" and self._estop_active:
            self._estop_active = False
            self.get_logger().info("固件报告急停已解除")

    def _on_odometry(self, fields):
        # O <millis> <x> <y> <yaw> <vx> <vy> <wz>
        if len(fields) != 8:
            return
        yaw = float(fields[4])
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._odom_frame
        message.child_frame_id = self._base_frame
        message.pose.pose.position.x = float(fields[2])
        message.pose.pose.position.y = float(fields[3])
        message.pose.pose.orientation.z = math.sin(yaw * 0.5)
        message.pose.pose.orientation.w = math.cos(yaw * 0.5)
        message.twist.twist.linear.x = float(fields[5])
        message.twist.twist.linear.y = float(fields[6])
        message.twist.twist.angular.z = float(fields[7])
        # 编码器推算的位姿，协方差按经验值给出，供 EKF 参考
        message.pose.covariance[0] = 0.02
        message.pose.covariance[7] = 0.02
        message.pose.covariance[35] = 0.05
        message.twist.covariance[0] = 0.02
        message.twist.covariance[7] = 0.02
        message.twist.covariance[35] = 0.05
        self._odom_pub.publish(message)
        self._counts["odom"] += 1

    def _on_imu(self, fields):
        # I <millis> <ax> <ay> <az> <gx> <gy> <gz> <temp>，固件已使用 m/s² 与 rad/s
        if len(fields) != 9:
            return
        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._base_frame
        # 固件不做姿态解算，用 -1 标记 orientation 不可用
        message.orientation_covariance[0] = -1.0
        message.linear_acceleration.x = float(fields[2])
        message.linear_acceleration.y = float(fields[3])
        message.linear_acceleration.z = float(fields[4])
        message.angular_velocity.x = float(fields[5])
        message.angular_velocity.y = float(fields[6])
        message.angular_velocity.z = float(fields[7])
        for index in (0, 4, 8):
            message.angular_velocity_covariance[index] = 0.02
            message.linear_acceleration_covariance[index] = 0.2
        self._imu_pub.publish(message)
        self._counts["imu"] += 1

    def _on_lift(self, fields):
        # L <outputsEnabled> <configured> <ready> <active> <target> <pulse>
        if len(fields) != 7:
            return
        active = fields[4] == "1"
        target = int(fields[5])
        state = LIFT_MOVING if active else (
            LIFT_RAISED if target == 1 else LIFT_LOWERED)
        self._lift_pub.publish(UInt8(data=state))
        self._counts["lift"] += 1

    def _send(self, command):
        with self._write_lock:
            connection = self._connection
            if connection is None or not connection.is_open:
                return False
            try:
                connection.write((command + "\n").encode("ascii"))
            except (serial.SerialException, OSError) as error:
                if not self._link_error_reported:
                    self._link_error_reported = True
                    self.get_logger().error(f"串口写入失败：{error}")
                return False
        return True

    # ------------------------------------------------------------------ 下行命令

    def _on_cmd_vel(self, message):
        if self._estop_active:
            # 急停期间不让新的速度指令把车重新带起来
            self.get_logger().warn(
                "急停激活，忽略 /cmd_vel", throttle_duration_sec=2.0)
            return
        velocity = (clamp(message.linear.x, self._max_linear),
                    clamp(message.linear.y, self._max_linear),
                    clamp(message.angular.z, self._max_angular))
        with self._command_lock:
            self._pending_velocity = velocity

    def _on_lift_cmd(self, message):
        keyword = message.data.strip().lower()
        command = {"up": "lift up", "raise": "lift up", "raised": "lift up",
                   "down": "lift down", "lower": "lift down", "lowered": "lift down",
                   "stop": "lift stop"}.get(keyword)
        if command is None:
            self.get_logger().warn(
                f"未知的升降指令 {message.data!r}，仅支持 up/down/stop")
            return
        self._send(command)

    def _on_estop(self, message):
        active = bool(message.data)
        if active == self._estop_active:
            return
        if self._send("estop 1" if active else "estop 0"):
            self._estop_active = active
            self.get_logger().warn("已下发急停" if active else "已解除急停")

    # ------------------------------------------------------------------ 定时器

    def _on_control_timer(self):
        now = time.monotonic()
        with self._command_lock:
            pending = self._pending_velocity
            self._pending_velocity = None
        if pending is not None:
            if now - self._last_send >= self._min_command_period:
                if self._send("vel %.4f %.4f %.4f" % pending):
                    self._last_send = now
                    self._last_drive = now
                    self._zero_sent = False
            else:
                # 频率高于串口容忍度时保留最新一帧，由下一个周期发出
                with self._command_lock:
                    if self._pending_velocity is None:
                        self._pending_velocity = pending
            return
        if (not self._zero_sent and not self._estop_active and
                now - self._last_drive > self._command_timeout):
            if self._send("vel 0.0000 0.0000 0.0000"):
                self._zero_sent = True
                self.get_logger().warn(
                    f"{self._command_timeout:.1f}s 未收到 /cmd_vel，已补发零速")

    # ------------------------------------------------------------------ 退出

    def shutdown(self):
        self._running.clear()
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
        if self._connection is not None and self._connection.is_open:
            # 退出前明确停车，比依赖固件超时更快
            self._send("stop")
        if self._reader is not None:
            self._reader.join(timeout=1.0)
        if self._connection is not None and self._connection.is_open:
            try:
                self._connection.close()
            except (serial.SerialException, OSError):
                pass
        self.get_logger().info(
            "桥接已关闭：里程计 %d 帧，IMU %d 帧，升降 %d 帧，ACK %d，拒绝 %d" %
            (self._counts["odom"], self._counts["imu"], self._counts["lift"],
             self._counts["ok"], self._counts["err"]))


def main(argv=None):
    rclpy.init(args=argv)
    bridge = None
    exit_code = 0
    try:
        bridge = SerialBridge()
        if not bridge.start():
            exit_code = 1
        else:
            rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        if bridge is not None:
            bridge.shutdown()
            bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
