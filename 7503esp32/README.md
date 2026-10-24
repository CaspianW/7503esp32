# 7503 ESP32-S3 智能小车固件

在 Arduino IDE 中打开本目录的 `7503esp32.ino`。开发板选择 **ESP32S3 Dev Module**，
ESP32开发板包固定 **2.0.17**，通过库管理器安装TDK的 **ICM42670P 1.0.9**。
使用板上的UART0 USB转串口接口，关闭USB CDC On Boot；本次验证配置为4MB Flash、PSRAM禁用。
烧录前确认实际板型及Flash配置，不要直接升级到Arduino-ESP32 3.x，LEDC/PCNT接口不同。

## 目录

- `src/config`：板卡引脚、轮位映射、尺寸、PID、速度上限及输出开关。
- `src/drivers`：电机PWM、PCNT编码器、ICM42670P与升降舵机。
- `src/control`：麦克纳姆解算、测速、四轮PID、里程计与升降端点控制。
- `src/app`：100Hz调度、使能、超时、急停与故障处理。
- `src/communication`：115200 baud ASCII串口指令及10Hz状态回传；原生micro-ROS接口仍未实现。
- `tests`：在电脑上运行的C++硬件边界与逻辑测试，不参与Arduino固件构建。

## 上电与标定

默认 `motorOutputsEnabled`、`motorBenchEnabled`、`chassisControlEnabled`、
`wheelMappingConfirmed`、`liftOutputsEnabled` 均为false，上电不产生运动命令。
当前轴距配置为0.142m、轮距为0.17m；整车速度上限和PID增益仍为0，必须完成实车标定后再启用。

逻辑轮序为左前FL、左后RL、右前FR、右后RR。`wheelChannel` 把逻辑轮映射到电气M1～M4的0～3；
`motorDirection`、`encoderDirection` 按电气通道排列，填写+1或-1。
轮径0.065m和每输出轴转1040计数只是当前标称值，需核实AB四倍频和减速比。

单电机台架：同时启用 `motorOutputsEnabled` 和 `motorBenchEnabled`，关闭整车闭环开关。
上电等待至少2秒，所选 `motorBenchChannel` 以绝对值不超过0.2的占空比运行不超过500ms，然后停机。
每次上电只运行一次，stop或急停终止后不再续转。测试前保持车轮悬空。

闭环底盘：关闭台架，完成方向和轮位确认，填写实际尺寸、速度上限和PID，才可启用整车开关。
命令超过500ms未刷新会停机并解除使能；超过50ms的调度间隔、编码器读取错误或异常反馈会锁定故障。
故障需排除原因后复位，急停解除也不会自动恢复运动。

升降默认使用 **S1/GPIO8**，`liftServoChannel=0`；选择1时使用S2/GPIO21。
填写已标定的 `liftLoweredPulse`、`liftRaisedPulse`，两者须在500～2500µs内且不同，再启用 `liftOutputsEnabled`。
升降可以独立于底盘使能。停止升降PWM可能释放保持力，不保证带载悬停；没有高度反馈。

## 串口指令

UART0，115200，8N1，ASCII，每条指令以换行结束；单行最多95字符。

| 指令 | 行为 |
| --- | --- |
| `status` | 请求状态及测量 |
| `arm` | 配置有效时使能底盘，清除旧速度 |
| `vel vx vy wz` | 车体速度：m/s、m/s、rad/s，x向前、y向左、逆时针为正；零速度立即停底盘 |
| `stop` | 停底盘、关闭升降PWM并解除两者使能 |
| `estop 1` / `estop 0` | 急停 / 请求解除急停；解除后仍需新使能和目标 |
| `lift arm` | 使能升降，本身不输出位置脉冲 |
| `lift up` / `lift down` | 请求已标定的抬升 / 下降端点 |
| `lift keep` | 刷新独立的升降心跳，不延长底盘速度有效期 |
| `lift stop` | 关闭升降PWM并解除升降使能 |

成功回复 `OK`，拒绝回复 `ERR command_or_state`，无效帧回复 `ERR frame`。
升降使能后需持续发送新目标或 `lift keep`，500ms无心跳会关闭PWM；恢复心跳不会自动重启。

每100ms回传五行：

```text
S uptime initialized armed allowed estop fault encoder imu speed bench outputs chassis
E ticksM1 ticksM2 ticksM3 ticksM4 speedFL speedRL speedFR speedRR dutyFL dutyRL dutyFR dutyRR
O timestamp x y yaw vx vy wz
I timestamp ax ay az gx gy gz temperature
L enabled configured armed active target pulse
```

时间为MCU毫秒，不是ROS时间；计数为64位累计值，轮速m/s、位置m、角度rad、加速度m/s²、角速度rad/s。
升降target是指令目标：0=下降、1=抬升、2=未知/无输出，pulse单位µs，不是实测位置。
故障码：0正常、1配置、2电机PWM、3编码器、4调度超时、5反馈、6IMU、7升降初始化。

## 验证范围

默认诊断和底盘/升降同时启用的临时测试配置均已通过ESP32-S3交叉编译；19组C++逻辑及硬件边界测试通过。
尚未实车验证轮位、计数精度、PID、供电、IMU坐标和升降限位。IMU数据仍为芯片坐标，默认里程计航向使用轮速。
PCNT读取清零存在暂停计数窗口；软件停机依赖调度，主循环完全卡死时仍需独立断电手段。
