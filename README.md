# 7503 ESP32-S3 小车电控固件

本仓库是四轮麦克纳姆小车的 ESP32-S3 下位机代码，使用 Arduino IDE 编译和烧录。固件负责读取编码器与 IMU、计算轮速和里程计、控制四路轮电机及升降舵机，并处理控制指令与安全状态。

当前运行入口是 ASCII 串口控制台；`src/communication/ros_bridge.*` 仅为 micro-ROS 接口预留，**尚未实现 ROS 2 通信**。

## 代码结构

```text
7503esp32/
├── 7503esp32.ino          # Arduino 草图入口：初始化、主循环、状态灯
├── src/
│   ├── app/               # RobotController：调度、模式切换、故障与安全状态
│   ├── common/            # 各层共用的数据类型、轮序和坐标定义
│   ├── communication/     # 串口命令解析、状态输出；micro-ROS 预留接口
│   ├── config/            # 板卡引脚与机器人参数
│   ├── control/
│   │   ├── include/       # 底盘、轮速、PID、里程计、升降控制接口
│   │   └── src/           # 对应控制算法实现
│   └── drivers/
│       ├── include/       # 电机、编码器、IMU、舵机硬件接口
│       └── src/           # 对应硬件驱动实现
├── tests/                 # 电脑端逻辑和硬件替身测试，不参与固件编译
├── tools/                 # 电脑端辅助工具，例如键盘控制脚本
└── COMMIT.md              # 提交信息约定
```

`src/config/board_pins.h` 管理 GPIO 与外设接口；`src/config/robot_config.h` 管理轮位映射、尺寸、方向、控制参数和功能开关。跨模块的数据结构集中在 `src/common/robot_types.h`，避免各层重复定义。

## 控制流程

1. `7503esp32.ino` 创建 `RobotController` 和 `SerialConsole`，在主循环中依次更新控制与串口。
2. `communication/` 将串口输入解析为底盘速度、升降或安全命令，交给 `app/`；`app/` 决定当前模式、调度频率和是否允许输出。
3. 底盘路径：`control/` 做麦轮逆解、轮速目标和 PID 计算，`drivers/motor_driver` 输出电机 PWM。
4. 反馈路径：`drivers/encoder` 读取计数，`control/wheel_speed` 换算轮速，`control/odometry` 估算位姿；`drivers/imu` 提供惯性测量。
5. 升降路径：`control/lift_controller` 确定目标脉宽，`drivers/servo_driver` 输出舵机 PWM。安全和故障状态统一由 `app/` 管理。

车体坐标遵循 ROS 常用约定：X 向前、Y 向左、绕 Z 轴逆时针为正。逻辑轮序固定为左前 FL、左后 RL、右前 FR、右后 RR；逻辑轮位到 M1～M4 电气通道的对应关系由 `wheelChannel` 配置。

## 开始阅读与编译

建议从 `7503esp32.ino` → `src/app/robot_controller.*` → `src/control/` → `src/drivers/` 阅读。修改硬件接线或车体参数时，先查看 `src/config/`，不要在控制算法里写死引脚或轮位。

在 Arduino IDE 中打开仓库根目录的 `7503esp32.ino`。已验证的编译环境是 **ESP32S3 Dev Module**、Arduino-ESP32 **2.0.17** 和 TDK **ICM42670P 1.0.9**；本板使用 UART0 USB 转串口，USB CDC On Boot 关闭。Arduino-ESP32 3.x 的 LEDC/PCNT 接口不同，升级前需要适配驱动代码。

`tests/` 包含电脑端 C++ 测试和键盘工具的 Python 测试；其中 `tests/fakes/` 模拟 Arduino 与外设，不会烧录到板上。`tools/keyboard_control.py` 是独立的上位机测试工具，不属于固件。当前串口协议的实现以 `src/communication/serial_protocol.*` 为准。

## ROS 2 桥接（树莓派侧）

`tools/ros_serial_bridge.py` 在树莓派上把上述 ASCII 串口协议翻译成 ROS 2 话题，不需要重新烧录固件：

| 方向 | 话题 | 消息类型 | 串口侧 |
| --- | --- | --- | --- |
| 下行 | `/cmd_vel` | `geometry_msgs/Twist` | `vel <vx> <vy> <wz>`，需先 `arm` |
| 下行 | `/lift_cmd` | `std_msgs/String` | `lift up` / `lift down` / `lift stop` |
| 下行 | `/estop` | `std_msgs/Bool` | `estop 1` / `estop 0` |
| 上行 | `/wheel/odom` | `nav_msgs/Odometry` | `O` 帧 |
| 上行 | `/imu/data_raw` | `sensor_msgs/Imu` | `I` 帧 |
| 上行 | `/lift_state` | `std_msgs/UInt8` | `L` 帧，0=落下、1=抬起、2=运动中 |

运行方式（Ubuntu 24.04 + ROS 2 Jazzy）：

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install python3-serial
python3 tools/ros_serial_bridge.py --ros-args -p port:=/dev/ttyUSB0
```

注意：

1. 桥接节点不发布 `odom` → `base_link` 变换，该变换由树莓派侧 EKF 唯一发布，避免双发布冲突；`/wheel/odom` 与 `/imu/data_raw` 作为 EKF 的输入。
2. 固件 `telemetryPeriod` 当前为 100 ms，因此上行话题的实际频率是 10 Hz。接口表要求的更高频率需要修改 `src/config/robot_config.h`，只改桥接脚本无效。
3. 固件在 500 ms 内收不到 `vel` 命令会锁停，桥接脚本在此基础上再补一层看门狗并主动发零速。
4. 打开串口会触发 ESP32 复位，脚本默认等待 2 s 启动时间后再发 `arm`；`arm` 未确认时脚本直接报错退出，不会静默继续。

> 这是课程项目固件，不是通用安全控制器。实车运行前仍需核对接线、轮位、转向和急停手段。
