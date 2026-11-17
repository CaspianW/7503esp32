# 分支与烧录

所有操作都在同一个目录 `/home/fdnan/HKU/7503/7503esp32` 中进行。切换后在 Arduino IDE 重新打开根目录的 `7503esp32.ino`，重新编译并烧录。仅切换 Git 分支不会更新板上的程序。

| 分支 | 用途 | 板载 Type-C serial 连接 |
| --- | --- | --- |
| main | 正式运行，二进制协议控制和遥测 | 树莓派 |
| feature/debug_motor | 手转车轮检查计数和轮速，底盘保持未使能 | 电脑 Arduino Serial Monitor，115200 |
| feature/keyboard_ctrl | 复用键盘脚本，通过文本控制台测试运动和升降 | 电脑终端，115200 |

三个分支均使用新接线 FL=M2、RL=M4、FR=M1、RR=M3，方向参数随原车轮迁移。升降保留上电下降行为。

## 键盘测试

关闭 Arduino Serial Monitor，释放串口。在工程目录执行：

```bash
git switch feature/keyboard_ctrl
# 先用 Arduino IDE 编译并烧录本分支
sudo apt install python3-serial python3-evdev
sudo python3 tools/keyboard_control.py --port /dev/ttyUSB0
```

默认复用原来的开环测试，活动轮占空比 70%，按 `-` / `=` 每次减 / 加 1%。需要更低占空比时加 `--duty 0.6`，低于电机起动阈值可能不转。

测试当前轮速闭环时运行（任一活动轮解算目标不得低于 `0.30 m/s`，对应单轴最低阈值 `|vx| >= 0.30 m/s`、`|vy| >= 0.30 m/s`、`|wz| >= 1.49 rad/s`）：

```bash
sudo python3 tools/keyboard_control.py --port /dev/ttyUSB0 --closed-loop --speed 0.3 --turn 2.0
```

若发送低于上述阈值的非零速度（例如 `--speed 0.1 --turn 0.5`），底盘会保持使能并停机等待，`S` 行持续上报错误码 `5`，直到收到正常速度指令后自动清零恢复。

W/S 前后，A/D 左右，Q/E 旋转，松开运动键停车。U 抬升，J 下降，空格零速度，T 急停 / 解除，R 重新使能，Esc 停车并退出。坐标遵循 ROS，x 前、y 左、z 上。脚本每 100 ms 刷新速度，固件 500 ms 无有效速度命令会停车并解除使能。

脚本通过 Linux evdev 读取实体键盘，需要本机键盘设备的权限。普通 SSH 终端输入不会成为远端 evdev 事件。多键盘时用 `--device /dev/input/by-id/…-event-kbd` 指定设备。启动会自动使能，但收到运动键前输出零速度。

S 行包含初始化、使能、故障等状态，其中 fault 是 S 行拆分后的第 7 项（索引 6），5 表示反馈故障。E 行依次为 FL/RL/FR/RR 的目标轮速、实际轮速、占空比。默认开环不执行闭环失速 / 反向反馈保护，排查 05 应使用 `--closed-loop`。

## 切换回其他用途

```bash
git switch main
# 重新烧录后，Type-C serial 接回树莓派

git switch feature/debug_motor
# 重新烧录后，打开 Serial Monitor 手转逐轮检查
```

有未提交修改时先提交或暂存，再切换。历史 stash 作为备份保留，当前各分支已包含新接线配置，无需为了换线再次应用旧 stash。
