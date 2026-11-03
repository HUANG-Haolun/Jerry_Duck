# ubt12hc_pd — UBT-12HC 舵机修改版固件

基于朋友的 `send_rev_230400_ult.ino`（仓库根目录，原版不动）改的。**通信协议完全不变**，主控端不用改。

## 改了什么

除了注释，实际代码只改了两处：

| | 原版 | 修改版 |
|---|---|---|
| 控制律 | `pwm = clamp(|err|, 48, 255)`，`|err| ≤ 2` 停 | `pwm = 16·|err| + 15`，上限 255，`|err| ≤ 1` 停 |
| 误差 10 计数（3.5°）时 | PWM 48（误差 1°~17° 一律 48） | PWM 175 |
| 通信超时 | 无（主控死机 → 舵机一直追最后目标） | 200 ms 没收到本 ID 的命令 → 失力（同 CMD 03） |

参数在文件开头，三个 `#define`：

```c
#define KP_PWM   16     // 比例刚度，PWM/计数
#define FF_PWM   15     // 静摩擦前馈
#define DEADBAND 1      // 死区，计数
#define COMM_TIMEOUT_MS 200UL   // 0 = 关闭
```

参数来源：仿真里按固件 1 kHz 复刻这条控制律（`microduck_rl/.../actuator/ubt12hc.py`，`JDUCK_FW=16,15,1`），
用官方规格（7.4 V：4.8 rad/s、0.96 N·m）训练，3 个种子里 2 个在 400 迭代内学会迈步；原版控制律 4000 迭代都不迈步。

验证情况：控制律和超时逻辑已在 PC 上用 gcc 单独编译测试（PWM 计算与仿真执行器逐项一致，含方向、溢出、`millis()` 溢出），
**但没有在 ATmega8 上编译、也没有上过硬件**。

## 烧录（每个舵机一个 ID，要逐个烧）

`SERVO_ID` 是编译期常量，所以先一次性生成 15 个版本：

```bash
python3 firmware/ubt12hc_pd/make_id_variants.py
# → firmware/ubt12hc_pd/build/id_01/id_01.ino … id_16/id_16.ino（没有 6，和 joint_limit.txt 一致）
```

每个版本和主文件只差 `#define SERVO_ID` 那一行。

**Arduino IDE 设置**（要和朋友当初的一致，最好问一下）：
- 开发板：MiniCore → ATmega8
- Clock：External 14.7456 MHz
- Bootloader：No bootloader
- 编程器：USBasp（或朋友用的那个）
- 上传：**项目 → 使用编程器上传**（`Ctrl+Shift+U`）

⚠ **不要点「烧录引导程序 / Burn Bootloader」**：它会改熔丝位，设错可能把芯片锁死。

## 推荐顺序：先一个，再全部

1. **只烧一个舵机**（比如膝关节的 ID 9），在台架上或者机器人吊起来测：
   ```bash
   python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 scan              # 应答正常
   python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 verify left_knee  # 方向对、能精确到目标
   python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 speed-test --id 9 # 测转速
   ```
   看三件事：
   - **到位精度**：`verify` 里实际值和目标差 ≤ 2 计数
   - **有没有抖/嗡嗡响**：上电保持时听一下、摸一下
   - **超时**：上电后停止发命令（Ctrl+C 退出程序），约 0.2 s 后应自动松掉
2. 有问题按下表调，改完重新生成 + 烧这一个：

   | 现象 | 调整 |
   |---|---|
   | 目标附近来回抖、嗡嗡响 | `KP_PWM` 降到 12；还抖就 `DEADBAND` 改 2 |
   | 停在离目标几个计数处不动 | `FF_PWM` 加到 20~25 |
   | 带负载（站立）时下沉很多 | `KP_PWM` 加到 20 |

   **改了参数要告诉 Claude**，仿真里要用同样的 `JDUCK_FW=K,FF,DB` 重新训练，否则仿真和真机不一致。
3. 一个舵机没问题了，再烧其余 14 个，**每烧一个就贴好 ID**，最后 `scan` 确认 15 个都应答。

## 注意

- `calibrate.py` 的 `verify` / `speed-test` 已经改成上电期间每 50 ms 重发一次目标，不会被 200 ms 超时打断。
  `monitor` / `joint` 全程卸力，不受影响。`jduck_run.py` 每 20 ms 发一次，也不受影响。
- 用老固件的舵机和新固件的舵机可以混在一条总线上（协议一样），但控制效果不同，训练要按新固件来。
