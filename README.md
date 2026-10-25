# 7503 ESP32-S3 智能小车固件

仓库根目录就是 Arduino 草图目录。在 Arduino IDE 中打开根目录的 `7503esp32.ino`；
不要再额外创建一层同名文件夹。开发板选择 **ESP32S3 Dev Module**，
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

`motorBenchEnabled` 已关闭；`chassisControlEnabled` 和 `manualOpenLoopEnabled` 均已启用，
分别由`arm`和`manual arm`选择闭环或开环，两种模式不会同时运行。轮位映射和升降输出已启用。
`encoderReadEnabled=true` 时仍会只读采样编码器，并在串口状态行回传计数和轮速，不改变开环电机输出。
当前轴距配置为0.142m、轮距为0.17m。根据70%占空比时四轮约0.63～0.69m/s的空载实测结果，
首轮数据确认0.3m/s目标可稳定在0.29452m/s。开环扫描确认四轮可靠起转阈值为0.60、共同维持阈值为0.55；
当前按0.55～0.75数据拟合`Ks=0.515、Kv=0.275`。为保持逻辑简单可靠，任何非零轮速目标的占空比绝对值都不低于0.60；零目标仍立即停止。最低实际轮速约为0.27～0.33m/s。
PID改为`Kp=0.4、Ki=0.5、Kd=0`；轮速变化上限设为50m/s²，使测试目标在一个控制周期内到位，
不再人为延迟电机起转。最大平移速度0.5m/s、最大轮速0.7m/s，仍需继续实车整定。

逻辑轮序为左前FL、左后RL、右前FR、右后RR。当前接线为 `wheelChannel={3,2,1,0}`：
FL=M4、RL=M3、FR=M2、RR=M1，`wheelMappingConfirmed=true`。
`motorDirection` 按电气通道M1～M4排列；目前按前进测试补偿左右电机镜像安装为`{-1,-1,1,1}`，轮位映射不能填进此数组。
`encoderDirection` 按M1～M4排列。根据本次手动使各轮沿车体正前方向转动的读数，原始计数符号为`{+,+,-,-}`，配置用`{1,1,-1,-1}`统一修正，因此正前方向的逻辑计数都应为正。每输出轴转数按1040 count/rev使用，忽略实测约1078的少量误差。

台架测试已关闭。若以后重新测试，M1～M4 会同时按 `motorBenchDuty=0.7` 持续输出。
发送 `stop` 会停底盘并关闭升降PWM；触发急停只停止底盘，升降不受影响。发生故障时底盘和升降PWM都会停机。

闭环底盘：完成轮位映射、电机方向和编码器方向校准，填写速度范围和PID后，才可启用整车开关。
命令超过500ms未刷新会停机并解除使能；超过50ms的调度间隔、编码器读取错误或异常反馈会锁定故障。
故障需排除原因后复位，急停解除也不会自动恢复运动。

升降舵机当前接在 **S2/GPIO21**，`liftServoChannel=1`。
当前 `liftLoweredPulse=1700µs` 为已测底部，`liftRaisedPulse=500µs` 为已测顶部。
上电时会初始化升降PWM，但未收到位置命令前保持0占空比；收到命令后持续输出目标脉宽，直到新目标或显式停止。底盘急停不会关闭升降PWM，也不影响升降命令。停止PWM可能释放保持力，不保证带载悬停；没有高度反馈。

端点使用已测的机械行程值，不在串口监视器中直接输入数字调脉宽；舵机只接受 `lift up` 和 `lift down` 两个端点命令。`lift stop` 会关闭PWM，之后的新位置命令会自动重新初始化。未填写有效端点时程序会拒绝启动升降输出。

## 串口指令

UART0，115200，8N1，ASCII，每条指令以换行结束；单行最多95字符。

| 指令 | 行为 |
| --- | --- |
| `status` | 请求状态及测量 |
| `arm` | 配置有效时使能底盘，清除旧速度 |
| `vel vx vy wz` | 车体速度：m/s、m/s、rad/s，x向前、y向左、逆时针为正；零速度立即停底盘 |
| `manual arm` | 仅当 `manualOpenLoopEnabled` 开启时使能串口键盘开环，启动时保持停止 |
| `manual duty value` | 设置本次开环测试占空比，范围大于0且不超过1 |
| `manual vel vx vy wz` | 按麦轮解算方向（x前、y左、z逆时针）；不是实际速度闭环，每个非零轮固定输出 `manualDuty` |
| `manual stop` | 立即停止并解除键盘开环使能 |
| `stop` | 停底盘并关闭升降PWM |
| `estop 1` / `estop 0` | 只停止底盘 / 请求解除底盘急停；升降不受影响，解除后底盘仍需新使能和目标 |
| `lift up` / `lift down` | 请求已标定的抬升 / 下降端点 |
| `lift stop` | 关闭升降PWM；下一条位置命令会重新初始化输出 |

成功回复 `OK`，拒绝回复 `ERR command_or_state`，无效帧回复 `ERR frame`。
升降目标不使用心跳或超时关闭；PWM会保持到下一条升降命令或显式停止。单独输入数字不是舵机命令，不会设置舵机角度。

## 键盘运动测试

WASD键盘工具位于 `tools/keyboard_control.py`，在Ubuntu终端运行：

```bash
sudo apt install python3-serial python3-evdev
sudo python3 tools/keyboard_control.py --port /dev/ttyUSB0 --closed-loop --speed 0.3
```

先关闭Arduino IDE串口监视器，避免占用同一串口。工具读取Linux键盘的按下/松开事件；
若键盘设备权限不足，可用 `sudo` 运行，或用 `--device /dev/input/by-id/...-event-kbd`
指定设备。`W/S`前进/后退，`A/D`左移/右移，`Q/E`逆时针/顺时针旋转；松开运动键立即发零速度，
空格立即清空全部方向并停车，`R`重新使能，`Esc`停止并退出。运动和空闲期间每100ms刷新心跳。
按 `T` 触发/解除底盘软件急停；急停只停底盘，不影响升降。解除后仍须按 `R` 重新使能。
闭环模式下W/S和A/D发送`--speed`指定的平移速度，固件上限为0.5m/s；Q/E发送`--turn`指定的角速度，默认2rad/s，对应四轮约0.312m/s。
省略`--closed-loop`时进入开环标定模式，`--duty`用于指定占空比，例如`--duty 0.55`。
开环运行期间可按`-`或`=`实时降低或提高占空比，每次变化0.01，用于测量起转后的最低维持占空比。
PID根据四轮目标和编码器实测速度计算占空比，输出绝对值不超过0.7。
空闲时工具也每100ms发送零指令心跳；`U`发送抬升命令、`J`发送下降命令，舵机保持最后脉冲直到下一条升降命令或停止；按 `Esc` 或关闭工具会发送`stop`。
`S`行的`chassis=1、manualMode=0`表示当前固件启用了闭环模式。

每100ms回传五行：

```text
S uptime initialized armed allowed estop fault encoder imu speed bench outputs chassis manualMode
E targetFL targetRL targetFR targetRR speedFL speedRL speedFR speedRR dutyFL dutyRL dutyFR dutyRR
O timestamp x y yaw vx vy wz
I timestamp ax ay az gx gy gz temperature
L enabled configured ready active target pulse
```

时间为MCU毫秒，不是ROS时间；`E`行依次显示四轮目标速度、实测速度和PID/开环占空比，轮序均为FL、RL、FR、RR。轮速单位m/s，占空比范围-1～1。位置单位m、角度单位rad、加速度单位m/s²、角速度单位rad/s。
当前`encoderReadEnabled=true`；编码器累计计数只在内部用于相邻采样测速和里程计，不再通过串口显示。轮径0.065m、半径0.0325m，按1040 count/rev换算时，每个计数对应约0.00019635m。
升降 `ready` 表示PWM已初始化；target是指令目标：0=下降、1=抬升、2=自定义脉宽或无输出，pulse单位µs，不是实测位置。
故障码：0正常、1配置、2电机PWM、3编码器、4调度超时、5反馈、6IMU、7升降初始化。

## 轮速闭环测试

先将四轮悬空，烧录后用带`--closed-loop`参数的键盘工具测试。持续按W约3秒，`E`行前四项应从0平滑增加到约0.5m/s，中间四项应跟随目标且符号同为正，最后四项是PID输出占空比。松开W后目标和占空比立即归零，实测速度随车轮停止回到零。再短按S验证四轮目标和实测速度同为负。

闭环控制使用相邻两次累计计数差计算轮速，不使用车轮绝对角度。100Hz控制周期下，每增加1 count对应约0.01963m/s的瞬时轮速分辨率；初次整定D项保持0，避免编码器量化使微分项抖动。PID输出上限为0.7，失速检测从实测起转占空比0.6开始计时。若`S`行故障码变为5，应立即松键并复位，检查对应轮目标、实测速度的符号和数值。

台架模式还会输出四行 `P channel pin duty freq route out gpio pause`，通道按 M1～M4 编号。
`duty` 是LEDC占空比回读值，`freq` 单位Hz；`route` 是GPIO矩阵实际信号编号，
`out` 是LEDC输出使能，`gpio` 是GPIO输出使能寄存器位，`pause` 是定时器暂停位。
当前四路正向100%的正常参考值为（Arduino-ESP32 2.0.17 将10位PWM满输出转换为1024）：
```text
P 1 5 1024 25000 73 1 1 0
P 2 16 1024 25000 74 1 1 0
P 3 10 1024 25000 75 1 1 0
P 4 14 1024 25000 76 1 1 0
```
这些是外设配置与寄存器回读，不是电机端电压或转速测量；停止时pin为-1。

## 验证范围

底盘与升降的实测验证状态以当前测试结果为准；C++逻辑及硬件边界测试位于 `tests`。
尚未实车验证轮位、计数精度、PID、供电、IMU坐标和升降限位。IMU数据仍为芯片坐标，默认里程计航向使用轮速。
PCNT读取清零存在暂停计数窗口；软件停机依赖调度，主循环完全卡死时仍需独立断电手段。
