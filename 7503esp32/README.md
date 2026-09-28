# 7503 ESP32 固件框架

在 Arduino IDE 中打开 `7503esp32.ino`。草图的 `setup()` 和 `loop()`
由 `RobotController` 负责组织；具体实现位于 `src/`，Arduino 构建系统会递归编译其中的源文件。

| 目录 | 职责 | 主要文件 |
| --- | --- | --- |
| `src/config/` | 开发板引脚、需要测量或调参的机器人参数 | `board_pins.h`、`robot_config.h` |
| `src/common/` | 各模块共用的数据类型和车轮顺序 | `robot_types.h` |
| `src/drivers/` | 用 Arduino／ESP32 接口读写硬件 | 电机驱动、编码器、IMU、舵机 PWM |
| `src/control/` | 控制算法和机构控制 | 通用 PID、四轮速度控制、麦克纳姆轮解算、里程计、升降控制 |
| `src/communication/` | 与树莓派通信的 USB 串口 micro-ROS 接口 | `ros_bridge.h/.cpp` |
| `src/app/` | 初始化、周期调度和运动安全 | 机器人控制器、安全逻辑 |

预期的底盘控制链路是：`cmd_vel` → 应用层 → 麦克纳姆轮逆运动学 → 四轮速度控制（调用通用 PID）→ 电机驱动。
编码器计数用于车轮速度闭环和里程计；IMU 提供角速度测量。通信模块向树莓派发布里程计和 IMU 数据。
升降链路是：升降目标 → 升降控制 → 舵机驱动。配发的 MG996 舵机没有向 ESP32 提供位置反馈，
因此 `src/config/robot_config.h` 中保存的是待实车标定的最低／最高控制脉宽，不是编码器值。

目前通用 PID（含 P/I/D、限幅和抗积分饱和）及四轮速度控制模块已经实现，
但尚未接入整车调度：`RobotController::begin()` 现在仍返回 `false`，固件还不能驱动车轮。
启用运动控制前，需要完成并验证驱动、底盘控制链路和 micro-ROS 通信，并测量或整定
`src/config/robot_config.h` 中的轴距、轮距、速度上限及 PID 参数。
升降端点脉宽当前均为零，升降控制仍为占位；舵机驱动会在端点未标定时拒绝启用。确认机构限位与供电前不要配置端点或启用升降。

目录划分参考了 Infantry RM STM32 项目中“设备、算法、应用”分层的思路。
Arduino-ESP32 已提供 GPIO、I2C、串口、PWM 和 FreeRTOS 支持，因此这里不需要移植 STM32 HAL 或整套中间件。

## 驱动层当前状态

使用 Arduino IDE 的 ESP32 开发板包 2.0.17，并通过库管理器安装 TDK/Invensense 的
`ICM42670P` 库 1.0.9。电机使用 LEDC 通道 0～3，每个电机按方向切换 PWM 到 A/B 引脚；
舵机使用通道 4。编码器使用 ESP32-S3 的四个 PCNT 单元进行 AB 相四倍频计数。
IMU 通过 GPIO40/39 的 I2C 读取，并自动检测 0x68/0x69 地址。

目前 `motorOutputsEnabled` 为 `false`，非零电机命令也不会输出；升降脉宽端点为零，
舵机驱动初始化会拒绝启用。实车装配后须逐轮确认电机和编码器方向、校准舵机端点，
再修改这些安全配置。`stopAll()` 当前只保证两路输入为低，不提供尚未核实的主动制动。
IMU 读数已转换为国际单位，但仍处于芯片坐标系；确定控制板安装朝向后，才能映射到 ROS 的 `base_link`。
固件尚未调用这些驱动，`RobotController::begin()` 仍返回 `false`。

## 源代码文件头规范

新增或修改 `.ino`、`.h`、`.cpp` 文件时，统一使用 Doxygen 文档块作为文件头，`@brief` 使用英文。固定填写 `@file`、`@brief`、`@author`、`@date`、`@version` 和 `@copyright`；作者为 `Caspian Wu`，日期采用 `YYYY-MM-DD`。摘要必须如实描述当前实现；未实现的功能应在英文摘要中标注为 `placeholder`。当前文件头版本统一为 `0.1.0`；许可证尚未确定前不添加 SPDX 标识。新代码请遵循本目录 `AGENTS.md` 中的模板。

```cpp
/**
 * @file 文件名.cpp
 * @brief Briefly describe the responsibility of this file.
 * @author Caspian Wu
 * @date YYYY-MM-DD
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */
```

## 代码变量命名规范

函数、局部变量、参数和配置项采用小驼峰命名法，例如 `readTicks`、`sampleCount`、`motorPwmFrequencyHz`；常量也采用小驼峰，不使用 `k` 前缀。类、结构体、枚举和类型别名采用大驼峰命名法，例如 `MotorDriver`、`ImuSample`。类的私有成员采用小驼峰并在末尾加 `_`，用于和局部变量区分，例如 `initialized_`。源文件名采用小写下划线分隔，例如 `motor_driver.cpp`。
