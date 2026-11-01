# ESP32 ↔ 树莓派串口协议

本仓库负责 ESP32 固件：采样、控制、串口收指令与发遥测。
树莓派串口读取、ROS 2 消息构造、话题发布和导航启动由上位机仓库维护。
本实现参考 feature/ros-serial-bridge 的字段和数据类型约定，继续使用 ASCII 协议。

## 板端实现位置

- src/communication/serial_protocol.*：校验并解析接收的一行控制命令。
- src/communication/serial_console.*：UART 初始化、有限字节预算接收、命令分发和周期发送。
- src/communication/serial_telemetry.*：从固定类型的 MCU 数据快照编码 S/E/O/I/L。
- src/app/robot_controller.*：控制器实际状态、速度与升降命令、安全超时。
- src/common/robot_types.h：Twist2D、OdometryState、ImuSample 的类型和单位。
- src/communication/ros_bridge.*：未启用的原生 micro-ROS 占位接口，不参与此串口链路。

7503esp32.ino 的 setup() 自动调用 console.begin()，loop() 自动执行控制与串口更新。
ESP32 上电后不等待树莓派连接，不等待 arm，也不需要另外启动通信程序。
arm 只使能底盘运动，和遥测是否发送无关。

## 物理和帧格式

- UART0，通过现有 USB-UART 转串口连接树莓派；USB CDC On Boot 关闭。
- 115200 baud，8 数据位，无校验，1 停止位（8N1）。
- 文本编码为 ASCII，空格分隔，每行 LF 换行；接收时忽略 CR。
- 一条下行命令最多 95 个字符，不含 LF。浮点采用点号小数。
- 整数和浮点都以十进制文本传输，不直接发送 C++ 内存结构，也不传 ROS 消息对象。
- 默认每 100 ms（10 Hz）发送一组状态。由 config::telemetryPeriod 配置。
- 里程计与 IMU 在板端按控制/采样周期更新，串口只发送当次快照。
- status 可主动请求一组状态，不改变采样时间和电机使能。

S/E/L 在周期内上报。O/I 为可选行：没有真实样本、传感器未就绪、非有限数或样本年龄达到
config::telemetrySampleMaxAge（默认 100 ms）时不发送对应行。
相比此前固定输出五行，字段顺序未变，但接收方不能假定每组一定有 O/I。
发送缓冲区不足时丢弃本次整组遥测，不等待，不影响控制循环；下个周期再次尝试。

## 上行：ESP32 发给树莓派

下表的字段数包含行首标签。bool 以 0/1 输出；float 为 C++ float 的十进制表示。

| 标签 | 字段数 | 内容 | 上位机转换目标（参考） |
| --- | --- | --- | --- |
| S | 14 | 板端时间、初始化/使能/故障/传感器状态 | 上位机状态监控 |
| E | 13 | 四轮目标速度、实测速度、输出占空比 | 调试；不是导航必需输入 |
| O | 8 | 真实采样时间、平面位置/航向、车体速度 | nav_msgs/msg/Odometry |
| I | 9 | 真实采样时间、三轴加速度、三轴角速度、温度 | sensor_msgs/msg/Imu |
| L | 7 | 升降配置、输出状态、命令目标和脉宽 | std_msgs/msg/UInt8 的目标值 |
| P | 9 | 台架模式电气通道和实际 PWM 外设状态 | 台架诊断 |

### S：状态

~~~text
S uptime_ms initialized armed motion_allowed estop fault encoder_ready imu_ready speed_ready bench motor_enabled chassis_enabled manual
~~~

- uptime_ms：uint32_t，板端 millis()，不是 ROS 时间。第一项数字不是 initialized。
- fault：uint8_t；0=无故障，1=配置，2=电机，3=编码器，4=控制时序，5=反馈，6=IMU，7=升降。
- 其余字段：bool，以 0/1 表示。
- armed=1 不代表正在运动；motion_allowed 还取决于新鲜速度指令、急停和安全状态。
- imu_ready=0 时，本周期无有效 I 行。O 的有效性还取决于里程计自身时间戳与有限值。

### O：轮里程计

~~~text
O sample_ms x y yaw vx vy wz
~~~

| 字段 | MCU 类型 | 单位 / 坐标 |
| --- | --- | --- |
| sample_ms | uint32_t | 真实采样时间，板端 ms |
| x, y | float | m，里程计坐标系中的位置 |
| yaw | float | rad，绕 Z 逆时针为正 |
| vx, vy | float | m/s，车体 X 向前、Y 向左 |
| wz | float | rad/s，绕车体 Z 轴 |

树莓派将 yaw 转为四元数，构造 Odometry；时间基准和坐标系名称由上位机处理。
ESP32 不发送 TF。导航当前由 EKF 唯一发布 odom→base_link 变换。

### I：原始 IMU

~~~text
I sample_ms ax ay az gx gy gz temperature
~~~

- sample_ms：uint32_t，真实采样时间，板端 ms。
- ax/ay/az：float，m/s²；gx/gy/gz：float，rad/s；temperature：float，°C。
- 数值为芯片原始坐标轴，驱动已完成 SI 单位换算。
- 不能把芯片坐标轴直接声称为已标定的 base_link。安装方向需实测后由约定的一方转换。
- 不发送姿态四元数或噪声协方差；这些不是本固件已经测得的数据。

### L：升降命令状态

~~~text
L outputs_enabled configured ready active target pulse_us
~~~

- 前四项：bool；ready 是输出模块可用，active 是当前有保持输出，均不代表到位。
- target：uint8_t，0=放下目标、1=抬起目标、2=无活动目标。
- pulse_us：uint16_t，命令 PWM 脉宽，单位 µs，无活动输出时为 0。
- 当前无到位传感器，target 不能作为“机械动作已完成”的证据。

### E / P：诊断

~~~text
E target_FL target_RL target_FR target_RR speed_FL speed_RL speed_FR speed_RR duty_FL duty_RL duty_FR duty_RR
P channel pin duty_raw frequency route out gpio pause
~~~

E 的目标/实测速率单位 m/s，占空比为无量纲比例，逻辑轮序 FL/RL/FR/RR。
P 只在台架模式额外发送，channel 为 1～4 电气通道；不是逻辑轮序。

## 下行：树莓派发给 ESP32

| 命令文本 | 含义 |
| --- | --- |
| status | 请求当前状态和有效遥测 |
| arm | 请求闭环底盘使能；仍需随后发送新的 vel |
| disarm | 只停底盘并丢弃旧速度，保留升降 PWM |
| vel vx vy wz | 车体速度，float，m/s / m/s / rad/s |
| estop 1 / estop 0 | 底盘急停 / 解除急停；解除后不自动 arm |
| lift up / lift down | 请求抬起 / 放下目标 |
| lift stop | 停止升降 PWM，可能释放保持力 |
| stop | 停底盘并停止升降 PWM |
| manual arm / manual stop | 原有开环调试命令 |
| manual vel vx vy wz / manual duty value | 原有开环方向和占空比调试 |

响应为 OK 或 ERR command_or_state；坏帧响应 ERR frame。
OK 表示命令已接收/被控制器接受，不代表升降到位。接收方还应观察 S/L 中的实际控制状态。
完整非法命令只拒绝；过长、NUL 或半帧超时沿用现有安全停机逻辑。
底盘在 500 ms 未收到有效速度指令后解除使能，旧指令不会自行恢复；重新运动需 arm 和新 vel。
串口断开没有单独物理握手，板端以该命令看门狗限制运动。

## 树莓派侧需遵守的边界

1. 逐行解析。S 第一项是 uptime，O/I 第一项是源采样时间；遵守字段顺序。
2. 源时间是 uint32_t millis()，约 49.7 天回绕。ROS header 时间需要上位机转换或同步。
   不要把板端毫秒直接填作 ROS epoch；不要把同一旧样本反复打成新采样时间。
3. O/I 缺失或 S 传感器状态失效时让对应数据过期，不用缓存重发维持“在线”。
   仅板端停止发送不能替代上位机的数据过期检查。
4. 将原始数值转换为 ROS 消息后，话题名、重映射、QoS 和导航入口由上位机仓库维护。
5. 原导航 launcher 引用本仓库 tools/ros_serial_bridge.py 的路径，需要上位机负责人迁移到其仓库。
   本仓库不再提供树莓派程序或 /ec/arm 等 ROS 服务，只提供表中的串口命令。

## 编译、测试与验收

Arduino IDE 打开根目录 7503esp32.ino。此链路不依赖 micro-ROS 库或 ROS 安装。
电脑端 C++ 回归（需要 g++，不参与烧录）：

~~~bash
bash tests/run_communication_tests.sh
~~~

覆盖字段顺序/数量/单位、启动后的自动周期发送、样本失效/非有限值/时间回绕、
缓冲区不足、命令解析、安全超时以及 disarm 保留升降保持力。
仍需用 ESP32 工具链编译烧录，并在实车连接树莓派验收波特率、方向、时延和停止行为。
