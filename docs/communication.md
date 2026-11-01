# 与现有导航栈对接的串口通信

本实现按导航仓库 main 的 521f12f 接口接入，不引入原生 micro-ROS。
ESP32 使用 SerialConsole，树莓派使用 Python 桥接；ros_bridge.cpp 继续保留给未来原生方案。
桥接依赖同目录的 serial_bridge_protocol.py，部署时必须一起提供。

## 接口和职责

下行：Nav2 / motion_guard → /cmd_vel → ec_adapter → /ec/cmd_vel → 串口 vel → RobotController。
上行：ESP32 O/I/L → Python 桥接 → /ec/... → ec_adapter → /odom/wheel、/imu/data_raw。
桥接不发布 odom→base_link TF；该变换仍由导航 EKF 唯一发布。

| 桥接原话题 | 导航 launcher 重映射后 | 类型 / 含义 |
| --- | --- | --- |
| /cmd_vel | /ec/cmd_vel | geometry_msgs/Twist，SI 单位，X 前、Y 左、Z 逆时针 |
| /wheel/odom | /ec/wheel/odom | nav_msgs/Odometry，odom / base_link |
| /imu/data_raw | /ec/imu/data_raw | sensor_msgs/Imu，原始芯片轴，无姿态 |
| /lift_cmd | /ec/lift_cmd | std_msgs/String，lift up/down/stop，兼容 up/down/stop |
| /lift_state | /ec/lift_state | std_msgs/UInt8，0/1 是目标，2 是无活动目标 |

兼容现有导航配置，IMU frame_imu 默认仍为 base_link，但这不代表轴向已标定。
部署时必须通过 NAV 的 imu_to_base_rpy_rad 实测配置旋转，或在完成板端轴变换后使用零旋转。
不能只改 frame 名而不转换数值；只在一处应用安装旋转。
角速度已经是 rad/s，加速度已经是 m/s²，不需要再次换算。

| 稳定的桥接服务名 | 类型 | 行为 |
| --- | --- | --- |
| /ec/arm | std_srvs/SetBool | true 请求 arm；false 发送 disarm |
| /ec/stop | std_srvs/Trigger | 发送 disarm，只停底盘并丢弃旧速度，保留升降 PWM |
| /ec/estop | std_srvs/SetBool | 设置板端底盘急停；解除后仍需显式 arm |

服务只有在收到新的、符合请求的 S 状态帧后才返回成功；发送失败、固件 ERR、复位或超时会返回失败。
/estop（std_msgs/Bool）仍兼容话题式请求，但没有确认响应；需要确认时使用服务。
升降动作不受底盘 arm 控制，与现有固件保持一致。
升降完成仍由导航 /ec_adapter/confirm_lift 人工确认；桥接不生成到位信号。

## 固件变化

新增 ASCII 指令 disarm，其余 S/E/O/I/L 格式保持不变：

- arm：闭环底盘使能，必须满足原有控制器条件。
- disarm：立即停底盘、解除使能、丢弃旧速度，保留升降保持输出。
- stop：保留旧语义，停底盘并停止升降 PWM。不要用于桥接退出或一般底盘解除使能。
- estop 1/0：仅作用于底盘；解除不恢复使能。
- lift up/down/stop：保留既有语义。

必须烧录包含 disarm 的本版本固件。旧固件遇到此指令会返回 ERR，不应作为本版本联调基线。
不需要更改 PID、轮位映射、串口波特率或遥测频率。
上电默认无底盘运动授权，串口无需等待树莓派就可以接收指令。

## 使用现有导航 launcher

先烧录固件，在树莓派准备 Ubuntu 24.04、ROS 2 Jazzy、python3-serial，并构建导航工作空间。
桥接独占串口，不要同时开启串口监视器、键盘工具或 micro-ROS Agent。

在导航仓库中启动无运动的通信调试：

~~~bash
bash scripts/nav.sh ec start_bridge:=true \
  bridge_root:=/实际路径/7503esp32 port:=/dev/ttyUSB0
~~~

该命令启动桥接与 ec_adapter；两个通信 Python 文件均来自 bridge_root 的 tools 目录。
现有 launcher 已包含五个原话题的重映射，不要再按固件 README 的独立运行命令另起一个节点。

查看链路：

~~~bash
ros2 topic echo /serial_bridge/status
ros2 topic echo /ec_adapter/status
ros2 topic echo /ec/wheel/odom --once
ros2 topic echo /ec/imu/data_raw --once
~~~

桥接状态中 connected 仅表示串口已打开；session_ready 表示启动后的解除使能同步完成；
sensors_fresh 表示对应真实采样有效；motion_authorized 表示成功的显式使能仍然有效。

需要联调空载导航时，按导航现有入口提供实测初始位姿：

~~~bash
bash scripts/nav.sh hardware navigation:=true \
  initial_x:=X initial_y:=Y initial_yaw:=YAW \
  start_bridge:=true bridge_root:=/实际路径/7503esp32 port:=/dev/ttyUSB0 \
  enable_motion:=true
~~~

X/Y/YAW 必须替换为实测 m/rad 数值。导航的 enable_motion 只允许速度转发，不会操作板端 arm。
传感器和初始位姿确认后，在同一 ROS 域显式使能：

~~~bash
ros2 service call /ec/arm std_srvs/srv/SetBool '{data: true}'
ros2 service call /ec/stop std_srvs/srv/Trigger '{}'
ros2 service call /ec/estop std_srvs/srv/SetBool '{data: true}'
ros2 service call /ec/estop std_srvs/srv/SetBool '{data: false}'
~~~

依次列出的命令用于使能、底盘停机、急停、解除急停；并非要求连续执行。
解除急停、固件 watchdog 停机或串口重连后，均需重新调用 arm 服务，不能重放旧速度。

串口打开后默认等待 startup_delay_s=2，再同步 disarm 状态。
启动时 arm_on_start 默认 false；显式开启也只尝试一次，重连或固件重启不自动恢复运动。
keepalive_period_s 默认 0；默认不为暂停的控制源持续发送保活指令。
桥接的 .25 s 速度超时发送一次零速，随后由板端 .5 s watchdog 解除使能。
导航 motion_guard 正常工作时会连续发零速，因此正常任务暂停不一定触发板端超时。

传感器以板端时间戳区分新样本，S 中的毫秒时间戳不会被当作 initialized；
源时间、板端就绪标志和串口接收时间共同决定有效性。
同一 O/I 样本不会重复发布，ROS 时间戳在收样时赋值一次，扣除 S 所示的板端采样年龄。
这不是硬件时钟同步，不能测出第一帧之前或 UART/USB 中未知的排队延迟；实际延迟仍需联调测量。
整条串口断开、板端时钟倒退（重启）或传感器过期时撤销主机运动授权。

## 当前仍需完成的集成与实测

1. 导航的 system.launch.py（nav.sh robot）尚未包含 ec_bridge.launch.py。
   正式整机入口应先启动桥接/适配器，再根据连接、传感器与使能状态放行任务。
   本仓库不修改导航仓库，也没有安装开机启动服务。
2. 导航 ec/firmware 子模块仍固定旧提交 2b56582。合并本变更后，更新该指针到包含两个桥接文件和新固件的提交。
3. 实车验证 IMU 的安装旋转、左转 gyro Z 的正方向、轮里程计符号、时间戳与真实串口吞吐。
   当前导航只融合 IMU yaw rate，仍需完成轴向与新鲜度验收。
4. 现有舵机只有目标脉宽，无升降到位传感器。自动取放需要双方确认到位依据和完成接口；
   当前人工确认方式不能作为无人干预的比赛自动完成。
5. 在真实 ROS/DDS 与硬件上验证首次使能、拒绝使能、停止、急停、拔线、板端复位、重连和恢复。
   host 回归不代表 ESP32 工具链编译、ROS 发现或实体硬件验收。

## 回归验证

~~~bash
python3 tests/run_communication_tests.py
~~~

需要 Python 3 和 g++，不需要 ROS、pyserial 或硬件。
测试包含 C++ 控制/串口回归、Python 桥接的消息与串口替身回归，
并将真实 C++ SerialConsole 输出送入 Python Telemetry，验证跨语言帧格式一致。
