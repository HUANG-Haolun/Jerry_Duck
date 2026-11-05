# ubt12hc_pd — UBT-12HC 舵机修改版固件

基于朋友的 `send_rev_230400_ult.ino`（仓库根目录，原版不动）改的。**通信协议完全不变**，主控端不用改。

## 改了什么

| | 原版 | 修改版 |
|---|---|---|
| 控制律 | `pwm = clamp(|err|, 48, 255)`，`|err| ≤ 2` 停 | `pwm = 16·|err| + 15`，上限 255，`|err| ≤ 2` 停 |
| 误差 10 计数（3.5°）时 | PWM 48（误差 1°~17° 一律 48） | PWM 175 |
| 通信超时 | 无（主控死机 → 舵机一直追最后目标） | 200 ms 没收到本 ID 的命令 → 失力（同 CMD 03） |
| 参数 | 写死在代码里 | **运行时可调，存 EEPROM**（新增 CMD 0x10/0x11/0x12） |

## 调参数：不用重新烧录

4 个参数（KP / FF / DEADBAND / 超时）可以直接通过总线改：

```bash
python3 jduck_tools/servo_params.py --port /dev/ttyUSB0 get --all                    # 看全部舵机当前参数
python3 jduck_tools/servo_params.py --port /dev/ttyUSB0 set --id 9 --db 1            # 只改 RAM，立即生效，断电丢失
python3 jduck_tools/servo_params.py --port /dev/ttyUSB0 set --id 9 --db 1 --save     # 同时存 EEPROM
python3 jduck_tools/servo_params.py --port /dev/ttyUSB0 set --all --kp 16 --ff 0 --db 0 --save   # 全部恢复成训练用的值
```

| 参数 | 默认 | 范围 | 含义 |
|---|---|---|---|
| `--kp` | 16 | 1–64 | 比例刚度，PWM / ADC 计数（原版相当于 1） |
| `--ff` | 0 | 0–120 | 静摩擦前馈，误差超出死区就加上 |
| `--db` | 0 | 0–10 | 死区，计数（1 计数 ≈ 0.35°） |
| `--timeout` | 20 | 0–255 | 通信超时，单位 10 ms（20 = 200 ms，0 = 关闭） |

上电时从 EEPROM 读；EEPROM 没存过、校验不对、或数值超出上面的范围，就用默认值（代码开头的 `DEFAULT_*`）。
**之前用 `--save` 存过参数的舵机会继续用 EEPROM 里的旧值**，烧了新固件也一样，要用上面最后一条命令重新设一次。

默认值 K=16 FF=0 DB=0 与当前策略（r7/r8，`JDUCK_FW=16,0,0`）训练时一致。仿真里同一个策略换参数的结果：
DB=1、DB=2、FF=4 都能正常走和站（DB=2 站立时轻微晃）；**FF=8 站立明显抖**（目标 1.3°/步）。

命令格式（帧格式和原来一样 `FA AF ID CMD PosH PosL SUM ED`）：

| CMD | PosH | PosL | 回复的 Speed 字段 |
|---|---|---|---|
| `0x10` 设参数 | 参数号（0=KP 1=FF 2=DB 3=超时） | 值 | 设后的值 |
| `0x11` 读参数 | 参数号 | — | 参数值 |
| `0x12` 存 EEPROM | `0xA5` | `0x5A`（防误写） | 0 |

参数号非法或 0x12 校验字节不对时，Speed = -1，什么都不改。这三条命令不改变力矩状态，但会刷新通信超时计时。

参数来源：仿真里按固件 1 kHz 复刻这条控制律（`microduck_rl/.../actuator/ubt12hc.py`），用官方规格
（7.4 V：4.8 rad/s、0.96 N·m）训练。原版控制律 4000 迭代都不迈步；K=16 线性律能学会走。
FF=15/DB=2 能走但零速时一直抖；当前默认 FF=0/DB=0 + 旧 HOME 训练出的策略（r7/r8）三个种子都站得住不抖、能转弯。

验证情况：控制律、超时、参数命令、EEPROM 读写已在 PC 上用 gcc 单独编译测试（PWM 计算与仿真执行器逐项一致，含方向、溢出、`millis()` 溢出；EEPROM 校验错误/超范围时回退默认值），
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
2. 有问题按下表用 `servo_params.py set` 调（**不用重新烧录**），调好了加 `--save`：

   | 现象 | 调整 |
   |---|---|
   | 目标附近来回抖、嗡嗡响 | `--db 1`（当前策略不用重练） |
   | 停在离目标几个计数处不动 | `--ff 4`（当前策略不用重练） |
   | 还不行：要 `--db 2` 以上、`--ff 8` 以上，或改 `--kp` | 改，然后**告诉 Claude 重新训练** |

   仿真里要用同样的 `JDUCK_FW=K,FF,DB` 训练，否则仿真和真机不一致。
3. 一个舵机没问题了，再烧其余 14 个，**每烧一个就贴好 ID**，最后 `scan` 确认 15 个都应答。
   如果第 2 步调过参数，烧完用 `servo_params.py set --all ... --save` 一次性统一设好。

## 注意

- `calibrate.py` 的 `verify` / `speed-test` 已经改成上电期间每 50 ms 重发一次目标，不会被 200 ms 超时打断。
  `monitor` / `joint` 全程卸力，不受影响。`jduck_run.py` 每 20 ms 发一次，也不受影响。
- 用老固件的舵机和新固件的舵机可以混在一条总线上（协议一样），但控制效果不同，训练要按新固件来。
