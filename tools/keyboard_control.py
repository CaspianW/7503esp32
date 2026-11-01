#!/usr/bin/env python3
"""@file keyboard_control.py
@brief 通过Linux键盘串口发送底盘开环和升降指令。
@author Caspian Wu
@date 2026-10-24
"""

import argparse
import math
import sys
import time


def parse_args():
    parser = argparse.ArgumentParser(
        description="用键盘控制小车底盘和升降机构；松开方向键立即停车。"
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="ESP32串口，默认 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200, help="串口波特率，默认 115200")
    parser.add_argument("--device", help="键盘输入设备，如 /dev/input/by-id/...-event-kbd")
    parser.add_argument(
        "--closed-loop", action="store_true",
        help="发送底盘轮速闭环的 arm/vel 命令；默认使用手动开环命令"
    )
    parser.add_argument(
        "--speed", type=float, default=0.3,
        help="闭环平移速度，单位m/s，默认0.3"
    )
    parser.add_argument(
        "--turn", type=float, default=2.0,
        help="闭环旋转角速度，单位rad/s，默认2.0"
    )
    parser.add_argument(
        "--duty", type=float, default=0.7,
        help="开环测试占空比，范围0～1，默认0.7"
    )
    parser.add_argument("--log-odom", action="store_true", help="显示里程计O行")
    parser.add_argument("--log-imu", action="store_true", help="显示IMU的I行")
    return parser.parse_args()


def should_log(line, log_odom, log_imu):
    return (line.startswith(("S ", "E ")) or
            (log_odom and line.startswith("O ")) or
            (log_imu and line.startswith("I ")))


def main():
    args = parse_args()
    if args.baud <= 0:
        print("波特率必须是正数。", file=sys.stderr)
        return 2
    speed = getattr(args, "speed", 0.3)
    turn = getattr(args, "turn", 2.0)
    duty = getattr(args, "duty", 0.7)
    log_odom = getattr(args, "log_odom", False)
    log_imu = getattr(args, "log_imu", False)
    if not math.isfinite(speed) or speed <= 0.0:
        print("闭环平移速度必须是正数。", file=sys.stderr)
        return 2
    if not math.isfinite(turn) or turn <= 0.0:
        print("闭环旋转角速度必须是正数。", file=sys.stderr)
        return 2
    if not math.isfinite(duty) or duty <= 0.0 or duty > 1.0:
        print("开环占空比必须大于0且不超过1。", file=sys.stderr)
        return 2

    try:
        import serial
        from evdev import InputDevice, ecodes, list_devices
    except ImportError as error:
        print("缺少依赖，请运行：sudo apt install python3-serial python3-evdev", file=sys.stderr)
        print(f"导入失败：{error}", file=sys.stderr)
        return 2

    key_names = ("KEY_W", "KEY_S", "KEY_A", "KEY_D", "KEY_Q", "KEY_E",
                 "KEY_U", "KEY_J", "KEY_MINUS", "KEY_EQUAL",
                 "KEY_SPACE", "KEY_R", "KEY_T", "KEY_ESC")
    missing = [name for name in key_names if not hasattr(ecodes, name)]
    if missing:
        print(f"当前evdev缺少按键定义：{', '.join(missing)}", file=sys.stderr)
        return 2

    required_keys = {getattr(ecodes, name) for name in key_names}

    def open_keyboard():
        if args.device:
            device = InputDevice(args.device)
            if not required_keys.issubset(set(device.capabilities().get(ecodes.EV_KEY, []))):
                device.close()
                raise RuntimeError("指定设备不包含完整的W/S/A/D/Q/E/U/J/空格/R/T/Esc按键。")
            return device

        denied = []
        for path in list_devices():
            try:
                device = InputDevice(path)
                if required_keys.issubset(set(device.capabilities().get(ecodes.EV_KEY, []))):
                    return device
                device.close()
            except OSError as error:
                denied.append(str(error))
        if denied:
            raise PermissionError("无法读取键盘输入设备；请用sudo运行，或通过--device指定有权限的键盘。")
        raise RuntimeError("没有找到包含所需按键的键盘；请用--device指定 /dev/input 下的键盘设备。")

    keyboard = None
    connection = None
    armed = False
    emergency_stopped = False

    def send(line):
        connection.write((line + "\n").encode("ascii"))

    closed_loop = getattr(args, "closed_loop", False)
    arm_command = "arm" if closed_loop else "manual arm"
    stop_command = "stop" if closed_loop else "manual stop"
    velocity_command = "vel" if closed_loop else "manual vel"

    def read_reply(timeout, accept_armed_state=False):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = connection.readline().decode("ascii", errors="replace").strip()
            if not line:
                continue
            if line == "OK" or line.startswith("ERR"):
                print(f"ESP32: {line}")
                return line
            # 高频遥测可能使很短的 OK 回包丢失；S 行仍可确认手动模式已经生效。
            if accept_armed_state and line.startswith("S "):
                field = line.split()
                expected_manual = "0" if closed_loop else "1"
                if (len(field) >= 14 and field[3] == "1" and
                        field[13] == expected_manual):
                    mode = "轮速闭环" if closed_loop else "手动开环"
                    print(f"ESP32: 已由状态帧确认{mode}使能")
                    return "OK"
        return None

    try:
        keyboard = open_keyboard()
        print(f"键盘：{keyboard.path} ({keyboard.name})")
        # 独占串口；若监视器仍在后台打开，会直接报告占用而不是悄悄抢走回包。
        connection = serial.Serial(
            args.port, args.baud, timeout=0.05, write_timeout=0.2, exclusive=True
        )
        print(f"串口：{args.port}，等待ESP32启动…")
        time.sleep(2.0)
        connection.reset_input_buffer()

        reply = None
        for attempt in range(3):
            send(arm_command)
            reply = read_reply(1.5, accept_armed_state=True)
            if reply is not None:
                break
            if attempt < 2:
                print("未收到使能确认，正在重试…")
                connection.reset_input_buffer()
                time.sleep(0.2)
        if reply != "OK":
            if reply is None:
                print("连续3次没有收到ESP32的使能确认。", file=sys.stderr)
            else:
                print("手动开环未使能；确认已烧录支持键盘开环的固件。", file=sys.stderr)
            return 1
        armed = True

        if not closed_loop:
            send(f"manual duty {duty:.4f}")
            reply = read_reply(0.5)
            if reply != "OK":
                print("ESP32未接受开环占空比。", file=sys.stderr)
                return 1

        mode = "轮速闭环" if closed_loop else "手动开环"
        print(f"已使能键盘{mode}。W/S前后，A/D左右，Q/E旋转；U抬升，J下降；空格停，T急停/解除，R重新使能，Esc停止并退出。")
        if closed_loop:
            print(f"松开运动键立即停车；W/S和A/D为{speed:g}m/s，Q/E为{turn:g}rad/s。")
        else:
            print(f"松开运动键立即停车；每个活动轮固定输出{duty * 100:g}%占空比。")
            print("保持运动键时，按-降低、按=提高占空比，每次变化1%。")
        print("E行：目标轮速FL/RL/FR/RR，实测轮速FL/RL/FR/RR，占空比FL/RL/FR/RR。")
        if log_odom:
            print("O行：时间戳、x、y、yaw、vx、vy、wz；x/y为世界坐标，速度为车体坐标。")
        if log_imu:
            print("I行：时间戳、加速度xyz、角速度xyz、温度；均为IMU原始轴。")

        pressed = set()
        last_velocity = None
        last_sent = 0.0
        running = True

        def velocity():
            vx = int(ecodes.KEY_W in pressed) - int(ecodes.KEY_S in pressed)
            vy = int(ecodes.KEY_A in pressed) - int(ecodes.KEY_D in pressed)
            wz = int(ecodes.KEY_Q in pressed) - int(ecodes.KEY_E in pressed)
            scale = max(1.0, math.hypot(vx, vy))
            if closed_loop:
                return (vx * speed / scale, vy * speed / scale, wz * turn)
            return (vx / scale, vy / scale, wz)

        while running:
            import select

            ready, _, _ = select.select([keyboard.fd, connection], [], [], 0.02)
            now = time.monotonic()

            if keyboard.fd in ready:
                for event in keyboard.read():
                    if (event.type == ecodes.EV_SYN and
                            event.code == ecodes.SYN_DROPPED):
                        send(stop_command)
                        read_reply(0.2)
                        armed = False
                        pressed.clear()
                        last_velocity = None
                        last_sent = time.monotonic()
                        print("键盘输入事件丢失，底盘已停止；按R重新使能。")
                        continue
                    if event.type != ecodes.EV_KEY:
                        continue
                    key = event.code
                    is_down = event.value == 1
                    is_up = event.value == 0
                    if key == ecodes.KEY_ESC and is_down:
                        send("stop")
                        read_reply(0.2)
                        armed = False
                        running = False
                        break
                    if key == ecodes.KEY_SPACE and is_down:
                        pressed.clear()
                        last_velocity = (0.0, 0.0, 0.0)
                        send(f"{velocity_command} 0 0 0")
                        last_sent = time.monotonic()
                        print("已发送零速度")
                        continue
                    if key == ecodes.KEY_T and is_down:
                        pressed.clear()
                        armed = False
                        last_velocity = (0.0, 0.0, 0.0)
                        last_sent = time.monotonic()
                        if emergency_stopped:
                            send("estop 0")
                            reply = read_reply(0.5)
                            if reply == "OK":
                                emergency_stopped = False
                                print("软件急停已解除；按R重新使能。")
                            else:
                                print("未确认急停解除；保持停止状态。", file=sys.stderr)
                        else:
                            send("estop 1")
                            reply = read_reply(0.5)
                            if reply == "OK":
                                emergency_stopped = True
                                print("软件急停已触发。再次按T解除，再按R重新使能。")
                            else:
                                print("未确认急停已触发；控制端保持停止。", file=sys.stderr)
                        continue
                    if key == ecodes.KEY_R and is_down:
                        pressed.clear()
                        if emergency_stopped:
                            print("急停仍处于触发状态；按T解除后再按R。")
                            continue
                        send(arm_command)
                        reply = read_reply(0.5)
                        armed = reply == "OK"
                        last_velocity = None
                        last_sent = time.monotonic()
                        continue
                    if key == ecodes.KEY_U and is_down:
                        send("lift up")
                        print("已发送抬升命令")
                        continue
                    if key == ecodes.KEY_J and is_down:
                        send("lift down")
                        print("已发送下降命令")
                        continue
                    if (not closed_loop and is_down and
                            key in (ecodes.KEY_MINUS, ecodes.KEY_EQUAL)):
                        step = -0.01 if key == ecodes.KEY_MINUS else 0.01
                        next_duty = min(1.0, max(0.01, round(duty + step, 2)))
                        send(f"manual duty {next_duty:.4f}")
                        reply = read_reply(0.2)
                        if reply is None or reply == "OK":
                            duty = next_duty
                            print(f"开环占空比：{duty:.2f}")
                        else:
                            print("ESP32未确认新的开环占空比。", file=sys.stderr)
                        continue
                    if key in required_keys and key not in (
                            ecodes.KEY_SPACE, ecodes.KEY_R, ecodes.KEY_T, ecodes.KEY_ESC,
                            ecodes.KEY_U, ecodes.KEY_J,
                            ecodes.KEY_MINUS, ecodes.KEY_EQUAL):
                        if is_down:
                            pressed.add(key)
                        elif is_up:
                            pressed.discard(key)

            if connection in ready:
                while connection.in_waiting:
                    line = connection.readline().decode("ascii", errors="replace").strip()
                    if line.startswith("ERR"):
                        print(f"ESP32: {line}")
                        armed = False
                        pressed.clear()
                        last_velocity = None
                    elif should_log(line, log_odom, log_imu):
                        print(f"ESP32: {line}")

            if not running:
                break

            target = velocity()
            # 变化时立即发送；有按键或空闲时都每100ms刷新，避免空闲后自动失使能。
            if armed and (target != last_velocity or now - last_sent >= 0.1):
                send(f"{velocity_command} {target[0]:.4f} {target[1]:.4f} {target[2]:.4f}")
                last_velocity = target
                last_sent = now
            elif not armed:
                time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n收到中断，正在停止底盘。")
    except (OSError, RuntimeError, PermissionError) as error:
        print(f"键盘或串口错误：{error}", file=sys.stderr)
        return 1
    finally:
        if connection is not None and connection.is_open:
            try:
                send("stop")
            except (OSError, serial.SerialException):
                pass
            connection.close()
        if keyboard is not None:
            keyboard.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
