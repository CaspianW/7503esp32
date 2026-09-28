# 7503 ESP32 固件框架

在 Arduino IDE 中打开 `7503esp32.ino`。草图的 `setup()` 和 `loop()`
由 `RobotController` 负责组织；具体实现位于 `src/`，Arduino 构建系统会递归编译其中的源文件。

| 目录 | 职责 | 主要文件 |
| --- | --- | --- |
| `src/config/` | 开发板引脚、需要测量或调参的机器人参数 | `board_pins.h`、`robot_config.h` |
| `src/common/` | 各模块共用的数据类型和车轮顺序 | `robot_types.h` |
| `src/drivers/` | 用 Arduino／ESP32 接口读写硬件 | 电机驱动、编码器、IMU |
| `src/control/` | 尽量不依赖具体硬件的控制算法 | 车轮 PID、麦克纳姆轮解算、里程计 |
| `src/communication/` | 与树莓派通信的 USB 串口 micro-ROS 接口 | `ros_bridge.h/.cpp` |
| `src/app/` | 初始化、周期调度和运动安全 | 机器人控制器、安全逻辑 |

预期的控制链路是：`cmd_vel` → 应用层 → 麦克纳姆轮逆运动学 → 四路车轮 PID → 电机驱动。
编码器计数用于车轮速度闭环和里程计；IMU 提供角速度测量。通信模块向树莓派发布里程计和 IMU 数据。

目前这只是**代码框架**，还不能驱动车轮：`RobotController::begin()` 现在返回 `false`。
启用运动控制前，需要实现并验证驱动、控制算法和 micro-ROS 通信，并测量或整定
`src/config/robot_config.h` 中的轴距、轮距、速度上限及 PID 参数。

目录划分参考了 Infantry RM STM32 项目中“设备、算法、应用”分层的思路。
Arduino-ESP32 已提供 GPIO、I2C、串口、PWM 和 FreeRTOS 支持，因此这里不需要移植 STM32 HAL 或整套中间件。
