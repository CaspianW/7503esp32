# 7503 ESP32-S3 小车电控固件

本仓库是四轮麦克纳姆小车的 ESP32-S3 下位机代码，使用 Arduino IDE 编译和烧录。固件负责读取编码器与 IMU、计算轮速和里程计、控制四路轮电机及升降舵机，并处理控制指令与安全状态。

本仓库只负责 ESP32 侧的控制与 ASCII 串口通信：上电自动采样并上报状态，接收树莓派的控制指令。树莓派串口读取、ROS 2 消息转换与导航启动由上位机仓库维护；`src/communication/ros_bridge.*` 为未启用的原生 micro-ROS 预留接口。

## 代码结构

```text
7503esp32/
├── 7503esp32.ino          # Arduino 草图入口：初始化、主循环、状态灯
├── src/
│   ├── app/               # RobotController：调度、模式切换、故障与安全状态
│   ├── common/            # 各层共用的数据类型、轮序和坐标定义
│   ├── communication/     # 串口命令解析、遥测编码和 UART 收发
│   ├── config/            # 板卡引脚与机器人参数
│   ├── control/
│   │   ├── include/       # 底盘、轮速、PID、里程计、升降控制接口
│   │   └── src/           # 对应控制算法实现
│   └── drivers/
│       ├── include/       # 电机、编码器、IMU、舵机硬件接口
│       └── src/           # 对应硬件驱动实现
├── tests/                 # 电脑端逻辑和硬件替身测试，不参与固件编译
├── tools/                 # 原有电脑端键盘台架调试工具
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

## ESP32 串口通信

`7503esp32.ino` 在 `setup()` 初始化串口，在 `loop()` 调用 `SerialConsole::update()`。上电后默认每 100 ms（10 Hz）自动上报，无需等待树莓派连接或发送 `arm`。串口为 UART0，115200 8N1，ASCII 文本，以换行分帧。

| 方向 | 数据 / 命令 | 板端内容 | 树莓派可转换的 ROS 类型 |
| --- | --- | --- | --- |
| 上行 | `O` | 样本时间、x/y/yaw、vx/vy/wz | `nav_msgs/msg/Odometry` |
| 上行 | `I` | 样本时间、三轴加速度/角速度、温度 | `sensor_msgs/msg/Imu` |
| 上行 | `L` | 升降配置、输出、目标、脉宽 | `std_msgs/msg/UInt8` 目标值 |
| 上行 | `S` / `E` | 控制状态、故障与四轮诊断 | 状态监控 / 调试 |
| 下行 | `vel vx vy wz` | 有限数速度命令，运动前需 `arm` | 上位机可从 `geometry_msgs/msg/Twist` 提取 |
| 下行 | `lift up/down/stop` | 升降命令 | 上位机负责转换 |
| 下行 | `arm` / `disarm` / `estop 0/1` | 使能 / 只停底盘 / 急停 | 上位机负责转换 |

串口字段顺序沿用 feature 分支的数据约定。`serial_telemetry.*` 只编码真实、有限且新鲜的 O/I 样本，未初始化或年龄达到 100 ms 时省略对应行；S/E/L 仍会上报。样本时间保留板端 `millis()`，不会用发送时间冒充采样时间。完整字段类型、单位和接收方约定见 [串口协议说明](docs/communication.md)。

`arm` 只影响底盘运动。新增 `disarm` 立即停底盘并保留升降 PWM；原有 `stop` 仍会停止底盘和升降输出。500 ms 无有效速度指令时，底盘看门狗停机并解除使能。

电脑端固件通信回归（需要 g++，不需要 ROS 或树莓派）：

```bash
bash tests/run_communication_tests.sh
```

本仓库不再包含树莓派 ROS 桥接程序。原导航 launcher 中指向本仓库 `tools/ros_serial_bridge.py` 的路径，需要上位机负责人迁移到上位机仓库。

> 这是课程项目固件，不是通用安全控制器。实车运行前仍需核对接线、轮位、转向和急停手段。
