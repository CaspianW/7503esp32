# 7503 ESP32-S3 小车电控固件

本仓库负责四轮麦克纳姆小车的 ESP32 固件：编码器与 IMU 采样、轮速和里程计、四路电机与升降舵机控制，以及与树莓派的二进制串口收发。不依赖 ROS 或 micro-ROS 库。

## 硬件串口

Yahboom MicroROS-Board 支持普通 UART 通信。课程硬件材料第 2 页明确给出 USB-Serial 连接，第 13 页标出独立的 Type-C 串口，第 14 页给出 UART0：TX=GPIO43、RX=GPIO44。
树莓派 USB-A 通过数据线连接板上的 Type-C 串口，区别于板上的 Type-C 电源口。
固件明确使用 UART0 的 115200 / 8N1；Arduino IDE 的 USB CDC On Boot 关闭。

## 代码结构

~~~text
7503esp32.ino              # 上电初始化、控制与串口主循环
src/app/                  # RobotController：调度、使能、故障、安全
src/communication/
  serial_link.*           # UART 收发、命令执行、周期遥测
  serial_protocol.*       # 帧头、长度、CRC、帧尾、字节序和流式解析
  serial_telemetry.*      # 里程计、IMU、升降完成位、控制状态打包
src/common/robot_types.h # 控制与传感器数据类型
src/config/               # 板卡引脚、轮位、参数和采样频率
src/control/              # 麦轮逆解、轮速 PID、里程计、升降
src/drivers/              # 编码器、IMU、电机 PWM、舵机 PWM
tests/                    # 按 communication/control/drivers/app 分组的电脑端回归
~~~

板端上电后自动开始串口收发，每 100 ms 上报一次，不需要先使能底盘。
主循环先执行安全检查与控制，再按字节预算处理串口；发送缓冲区不足时跳过本次发送，不阻塞控制。
坐标 X 向前、Y 向左、Z 逆时针为正；轮序 FL/RL/FR/RR。IMU 数据仍为芯片轴，安装方向需实测。

## 通信和升降

包结构为 AA 55 / 类型 / 长度 / 数据 / CRC16 / 0D 0A。字段和类型见 [串口协议](docs/communication.md)。
发送里程计、六轴 IMU、一个升降完成位和精简状态，不发送温度或舵机脉宽。
升降收到目标立即输出已标定 PWM，3 秒后更新完成位（0=下降、1=抬升），计时不阻塞底盘控制。
串口升降控制只有下降/抬升两个目标；03/04/05 的使能切换、底盘急停与停车不切断升降 PWM，也不重启升降计时。
MG996 没有位置反馈，完成位是计时估计。底盘 500 ms 无有效速度指令会停机并解除使能。

## 编译与测试

Arduino IDE 打开根目录 7503esp32.ino。原工程使用 ESP32S3 Dev Module、Arduino-ESP32 2.0.17、TDK ICM42670P 1.0.9。
Arduino-ESP32 3.x 的 LEDC/PCNT 接口不同，升级前需要适配驱动。

电脑端回归（g++，不需要硬件）：

~~~bash
bash tests/run_tests.sh
~~~

17 个测试程序分为通信 4 个、控制 7 个、驱动 3 个、安全与调度 3 个。它们不参与烧录；修改相应模块或参数后可按组运行，见 [测试说明](tests/README.md)。测试不代替 ESP32 工具链编译、烧录与实板收发验证。
