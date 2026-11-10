# 电脑端回归测试

这些测试用替身模拟时钟、串口、PWM、编码器和 IMU，验证 ESP32 的 C++ 代码；不参与烧录。
每个文件是一个独立程序，便于定位失败模块。已移除旧 ASCII 键盘脚本及其 Python 测试。

| 目录 | 程序数 | 验证内容 |
| --- | --- | --- |
| communication/ | 5 | 二进制包、CRC、残包恢复、命令执行、遥测 |
| control/ | 7 | PID、麦轮逆解、轮速、里程计、升降与 3 秒计时 |
| drivers/ | 3 | 编码器累计、IMU 读数、电机 PWM |
| app/ | 3 | 看门狗、急停、故障锁定、控制器调度 |
| fakes/ | — | 测试用硬件替身，不是额外的测试程序 |

需要 g++ 和 Bash，无需 ROS、树莓派或实车：

```bash
bash tests/run_tests.sh                # 全部 18 个程序及草图入口编译
bash tests/run_tests.sh communication  # 也可选 control、drivers、app
```

编译产物放临时目录并自动清理。改通信后跑 communication，改控制参数后跑 control/app，合并前跑全部。
IMU 驱动测试使用实际驱动配合寄存器替身；其余集成测试使用静止 IMU 替身。
