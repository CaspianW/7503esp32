# 二进制串口协议

UART0：TX=GPIO43、RX=GPIO44，经板载 Type-C 串口连接树莓派。115200 / 8N1，默认 10 Hz 上报。
所有多字节数据均为小端；float32 为 IEEE-754。下表中的十六进制数就是实际发送的字节。

| 偏移 | 字段 | 类型 / 长度 | 值 |
| --- | --- | --- | --- |
| 0 | header | uint8[2] | AA 55 |
| 2 | type | uint8 | 包类型，见下表 |
| 3 | length | uint8 | data 字节数，最大 32 |
| 4 | data | uint8[length] | 按下表依次排列，不含结构体填充 |
| 4+length | crc | uint16 | CRC-16/Modbus，仅计算 type、length、data |
| 6+length | tail | uint8[2] | 0D 0A |

CRC 初值 FFFF，多项式 A001，低字节先发；总包长为 length+8。

| 方向 | type | length | data 字段顺序与类型 |
| --- | --- | --- | --- |
| 下行 | 01 | 12 | vx:float32, vy:float32, wz:float32（m/s, m/s, rad/s） |
| 下行 | 02 | 1 | lift:uint8（0=下降，1=抬升） |
| 下行 | 03 | 1 | arm:uint8（0=只停底盘，1=使能底盘） |
| 下行 | 04 | 1 | estop:uint8（0=解除底盘急停，1=急停） |
| 下行 | 05 | 0 | 无；停止底盘及升降 PWM |
| 下行 | 06 | 0 | 无；请求当前上行包 |
| 下行 | 07 | 0 | 无；只停升降 PWM |
| 上行 | 81 | 28 | sample_ms:uint32, x,y,yaw,vx,vy,wz:float32×6（ms,m,m,rad,m/s,m/s,rad/s） |
| 上行 | 82 | 28 | sample_ms:uint32, ax,ay,az,gx,gy,gz:float32×6（ms,m/s²×3,rad/s×3，芯片轴） |
| 上行 | 83 | 1 | lift_state:uint8（0=下降计时完成，1=抬升计时完成） |
| 上行 | 84 | 2 | flags:uint8, fault:uint8 |
| 上行 | F0 | 2 | command_type:uint8, result:uint8（0=接受，1=拒绝） |

flags：bit0 初始化完成，bit1 底盘使能，bit2 底盘急停，bit3 里程计有效，bit4 IMU 有效，bit5 升降标志有效。
fault：0 正常，1 配置，2 电机，3 编码器，4 时序，5 反馈，6 IMU，7 升降。

升降立即输出 PWM，满 3000 ms 更新完成位，期间保持上次值；重复同目标不重计时，停止则取消计时。
首次动作完成前不发 83。完成位为计时估计，不是角度回读。81/82 无新鲜有效样本时不发。
底盘运动前先使能；500 ms 无有效速度指令会停机并解除使能。F0 表示命令接受，不表示升降完成。

抬升命令完整示例：AA 55 02 01 01 10 50 0D 0A。
