# 7503 ESP32-S3 小车电控固件

本仓库是四轮麦克纳姆小车的 ESP32-S3 下位机代码，使用 Arduino IDE 编译和烧录。固件负责读取编码器与 IMU、计算轮速和里程计、控制四路轮电机及升降舵机，并处理控制指令与安全状态。

板端运行入口是 ASCII 串口控制台，ROS 2 通信由树莓派上的串口桥接提供；`src/communication/ros_bridge.*` 仅为原生 micro-ROS 接口预留。

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
├── tools/                 # 树莓派 ROS 串口桥接、电脑端键盘调试工具
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

`tools/ros_serial_bridge.py` 在树莓派上把 ASCII 串口协议翻译成 ROS 2 话题，协议校验与采样时间处理位于 `tools/serial_bridge_protocol.py`。此版本新增板端 `disarm` 指令，需要同时烧录本版本固件。完整对接、启动和实车待办见 [通信接入说明](docs/communication.md)。

| 方向 | 话题 | 消息类型 | 串口侧 |
| --- | --- | --- | --- |
| 下行 | `/cmd_vel` | `geometry_msgs/Twist` | `vel <vx> <vy> <wz>`，需先 `arm` |
| 下行 | `/lift_cmd` | `std_msgs/String` | `lift up` / `lift down` / `lift stop` |
| 下行 | `/estop` | `std_msgs/Bool` | `estop 1` / `estop 0` |
| 上行 | `/wheel/odom` | `nav_msgs/Odometry` | `O` 帧 |
| 上行 | `/imu/data_raw` | `sensor_msgs/Imu` | `I` 帧 |
| 上行 | `/lift_state` | `std_msgs/UInt8` | `L` 帧，0/1=放下/抬起目标，2=无活动目标；均非到位反馈 |

桥接另提供 `/ec/arm`（SetBool）、`/ec/stop`（Trigger，只停底盘）和 `/ec/estop`（SetBool）服务；成功表示收到板端状态确认。默认不自动使能，重连不会恢复使能。

运行方式（Ubuntu 24.04 + ROS 2 Jazzy）：

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install python3-serial
python3 tools/ros_serial_bridge.py --ros-args -p port:=/dev/ttyUSB0
```

注意：

1. 桥接节点不发布 `odom` → `base_link` 变换，该变换由树莓派侧 EKF 唯一发布，避免双发布冲突；`/wheel/odom` 与 `/imu/data_raw` 作为 EKF 的输入。
2. 固件 `telemetryPeriod` 当前为 100 ms，上行最多约 10 Hz；同一板端采样时间戳只发布一次，失效或零初始化 IMU 不作为有效测量发布。
3. 固件在 500 ms 内收不到 `vel` 命令会锁停，桥接脚本在此基础上再补一层看门狗并主动发零速。
4. 打开串口可能触发 ESP32 复位。桥接默认等待 2 s，再解除底盘使能并确认状态。传感器就绪后，通过 `/ec/arm` 显式使能；拒绝或确认超时会返回失败。参数 `arm_on_start=true` 只允许节点启动时尝试一次，不能用于自动断线恢复。
5. 配合导航时使用导航仓库的 `ec_bridge.launch.py` 重映射五个原始话题，不能同时另起一份未重映射的脚本。`enable_motion=true` 只开启导航的速度转发，不等于板端 `arm`。

无需 ROS／硬件的通信回归：`python3 tests/run_communication_tests.py`，需要 Python 3 和 g++。它覆盖板端控制与串口测试、桥接状态与服务测试，并使用真实 C++ 遥测格式交叉验证 Python 解析。

> 这是课程项目固件，不是通用安全控制器。实车运行前仍需核对接线、轮位、转向和急停手段。
