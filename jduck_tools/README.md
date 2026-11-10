# jduck_tools — 真机端：舵机、标定、IMU、控制程序

UBT-12HC 总线舵机（协议见 `~/Downloads/UBT12HC_总线舵机通讯协议说明.pdf`）和机器人本体的全部真机端代码。
不需要 ROS，也不需要 uv 环境。`sim_robot.py`、`hw_chain_eval.py` 等仿真测评要用到 mujoco，
需要在电脑上的 `third_party/microduck_rl` 下通过 `uv run` 运行。

## 在泰山派上跑（2026-11-10 起的标准方式）

**连板子**：板子没有网口；用 USB 转串口线接它的调试串口，电脑上（Ubuntu）不用装东西：
```bash
python3 -m serial.tools.miniterm /dev/ttyUSB0 1500000     # 回车出现 root@taishanpi；Ctrl+] 退出
# 或者 sudo apt install picocom; picocom -b 1500000 /dev/ttyUSB0   （Ctrl+A 再 Ctrl+X 退出）
```
串口同一时间只能被一个程序占用。往串口一次塞太多数据会把终端切进 Rockchip 的 FIQ 调试器（提示符 `debug>`），
输入 `console` 回车、再按两次 Ctrl+C 就回到正常终端。

**第一次（或重刷系统后）准备环境**，在板子上的仓库目录 `/root/Jerry_Duck` 里：
```bash
jduck_tools/setup_board.sh          # 建 /userdata/jduck-venv（离线装 numpy 1.26.4 / onnxruntime 1.20.1 / pyserial / pyyaml / smbus2），
                                    # 编译 IMU 库 libjduck_icm.so，自检读一次 IMU
```
wheel 默认取 `/userdata/jduck-runtime/a8df940/wheels`（14 个 aarch64 / cp311 的 wheel，35 MB），也可以传别的目录。
系统 python3 本身没有 numpy / onnxruntime，所以**一定要用这个 venv**，`run_robot.sh` 会自动找。

**启动**（机器人先扶正：倾斜超过保护角会直接卸力）：
```bash
jduck_tools/run_robot.sh                               # 默认 O1 模型（原版固件），键盘控制
jduck_tools/run_robot.sh -m models/xxx.onnx            # 换模型
jduck_tools/run_robot.sh --hold-home                   # 只移到 HOME 并保持，不跑策略（看 HOME 准不准）
jduck_tools/run_robot.sh --log /userdata/stand.npz     # 录每一拍数据，带回电脑给 jitter_compare.py / real_calib.py
jduck_tools/run_robot.sh -- --dry-run                  # 只读传感器、跑策略，不给舵机上电
jduck_tools/run_robot.sh -h                            # 全部选项
```
默认参数就是 2026-11-09 晚上手敲的那条命令：`/dev/ttyS3` 230460 波特、`imu_spi.yaml`、上电 10 s 移到 HOME、
策略 5 s 渐入、按键速度上限 vx 0.08 / vy 0.05 / wz 0.25、头颈分频 2、倾斜保护 20°。
键盘：方向键走，a / e 转，空格停，q 退出（会卸力）。

**硬件**（来自朋友的 `/root/duck_hw` 测试程序）：舵机总线 `/dev/ttyS3` 230460 8N1；IMU 是 **ICM-42688P**，
走 SPI `/dev/spidev3.0`（±1000 °/s，100 Hz），板子绕躯干 Z 轴装反了 180°，`imu_spi.yaml` 里 `axes: ["-x","-y","z"]`
已修正、`gyro_bias` 已标定；电池电压是 ADS1115（I2C-2，0x48，`/root/duck_hw/ads1115_bat` 读）。

| 文件 | 作用 |
|---|---|
| `servo_bus.py` | 协议编解码和半双工总线驱动 |
| `servo_params.py` | 读写修改版固件的 KP / FF / DB / 超时，并保存到 EEPROM |
| `calibration.py` | ADC 和 URDF 关节角的换算、`calibration.yaml` 的读写、把标定范围写回 URDF |
| `calibrate.py` | 标定命令行工具：scan / monitor / measure-k / joint / report / apply / verify / speed-test |
| `servo_map.yaml` | **关节名到舵机 ID 的对应**（来自朋友的表，要用 `monitor` 确认一遍） |
| `calibration.yaml` | 标定结果（目前是从 `joint_limit.txt` 导入的，零位还不可靠） |
| `joint_limit.txt` / `import_joint_limit.py` | 朋友测的 ID、初始位置和范围表，以及把它导入成 `calibration.yaml` 的脚本 |
| `imu_spi_board.py` / `imu_spi.yaml` / `icm42688/` | **机器人上的 IMU（ICM-42688P，SPI）**：Python 封装、轴向与零偏配置、C 读数库源码（编成 `libjduck_icm.so`） |
| `imu_lsm6dsv16x.py` / `imu.yaml` | 另一款 IMU（LSM6DSV16X，I2C）的驱动和互补滤波；`imu_spi_board.py` 复用它的滤波。`jduck_run.py` 按配置里的 `chip` 自动选驱动 |
| `jduck_run.py` | **50 Hz 真机控制程序**：读舵机和 IMU，拼 61 维观测，跑 ONNX，发目标；先慢慢移到 HOME。带 `--dry-run`、`--hold-home`、倾斜卸力、策略渐入、头颈分频、`--log` 录数据、`--getup-policy` 摔倒后切起身策略 |
| `run_robot.sh` / `setup_board.sh` | 板子上的启动脚本和一次性环境准备（见上） |
| `jitter_compare.py` | 真机 `--log` 录的零速数据和同一策略的仿真逐关节对比 |
| `real_calib.py` / `calib_stand_eval.py` | 用真机录的数据校准仿真里的舵机参数；在“校准仿真”里测零速和各方向走路（**上机前的门槛**） |
| `fault_sweep.py` / `fall_chain_eval.py` | 逐个模拟真机故障（IMU 轴反、接线错、零位偏…）看哪个能复现症状；摔倒爬起测评 |
| `sim_robot.py` | 仿真替身：用 MuJoCo 冒充舵机总线和 IMU，按固件控制律在 1 kHz 下仿真，让 `jduck_run` 不接硬件也能跑 |
| `hw_chain_eval.py` | 用 `jduck_run` 的控制循环加 `sim_robot` 测评策略。**判断能不能上机以它为准** |
| `test_tools.py` | 离线测试，不需要硬件：`env -u PYTHONPATH python3 -m pytest jduck_tools/test_tools.py -q` |

```bash
cd third_party/microduck_rl
uv run --frozen python ../../jduck_tools/hw_chain_eval.py jduck_xxx.onnx            # 零速抖动、走、转
uv run --frozen python ../../jduck_tools/sim_robot.py --policy jduck_xxx.onnx --vx 0.2   # 单条指令
```

> 2026-11-10 起已在真机上用（O1 / O2cal 站立、走路、录数据）。标定（`calibrate.py`）那一套还没在真机上做过，
> 现在的 `calibration.yaml` 是从朋友的 `joint_limit.txt` 导入的；`--hold-home` 下看 HOME 和仿真一致，零位基本可信。

## 换算关系

```
q(rad, URDF) = sign × (adc − adc_zero) × k
```
- `adc_zero`：关节在 URDF 0 位时的 ADC 读数
- `sign`：往 URDF 正方向转时 ADC 增大为 +1，减小为 −1
- `k`：每个 ADC 计数多少弧度（规格书标称 2π/1024 ≈ 0.00614，要实测）

## 标定流程（在 `rl_control/` 下运行，需要真机）

### 0. 准备
- 机器人吊起来或放平，舵机供电，USB 转 TTL 插电脑。
- `ls /dev/ttyUSB* /dev/ttyACM*` 找到串口名，下面以 `/dev/ttyUSB0` 为例。
- 先在 `python3 scripts/view_urdf.py jduck_description/urdf/jduck_description.urdf`（或 RViz）里看好
  **每个关节 0 位的样子和正方向**——截图留着对照。

### 1. 扫描总线
```bash
python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 scan
```
应该看到 14 个 ID 都应答。没有应答时依次检查：供电、接线、`--direction`（模块不能自动切收发
时试 `--direction rts` 或 `rts-inv`）、`--timeout 0.05`。

### 2. 确认 ID ↔ 关节
```bash
python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 monitor
```
全部卸力，用手逐个掰关节，看哪一行在变，把对应关系改进 `servo_map.yaml`。

### 3. 测 k（做一次）
```bash
python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 measure-k --id 3 --angle-deg 90
```
摆两个相差已知角度（默认 90°）的姿态各按一次空格。结果接近 0.35°/计数说明是 1024 计数/360°。

### 4. 逐个关节标定
```bash
python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 joint              # 全部
python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 joint left_knee    # 单个
```
每个关节三步，每步按空格确认：
1. 摆到 **URDF 0 位** → 记录零位
2. 往 **URDF 正方向** 转 20° 以上 → 判断方向
3. **慢慢扫过两端**（碰到机械限位或别的零件就停），回中间 → 记录范围

每标完一个关节就自动保存。`s` 跳过当前关节，`q` 退出。

### 5. 检查结果
```bash
python3 jduck_tools/calibrate.py report
```
注意 ⚠ 提示：
- **ADC 范围贴近 50/950**：固件会把目标钳在 50–950，把舵盘拆下转一个齿重装，再标这个关节
- **扫动时有跳变**：可能经过电位器盲区，同样挪齿
- **零位不在可用范围内**：零位或方向标错了

### 6. 上电验证方向（机器人吊起来！）
```bash
python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 verify left_knee
```
只给这一个舵机上电，在当前位置附近 ±5° 动一下。确认往 URDF 正方向走，否则重标这个关节。
建议 14 个都过一遍。

### 7. 写回 URDF，重新生成模型
```bash
python3 jduck_tools/calibrate.py apply        # 原 URDF 备份为 *.urdf.precal.bak
cd third_party/microduck_rl && uv run --frozen python ../../jduck_mjcf/build_mjcf.py
```
然后需要重找 HOME 站姿（`jduck_constants.py`）并重新训练。

**拆装过舵盘的关节必须重新标定。**

## 测关节静摩擦

仿真里的关节摩擦（`JDUCK_FRICTION`）对能不能学会迈步、站着抖不抖影响很大，要用实测值。

**修改版固件**（精确）：
```bash
python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 friction --id 9
```
临时把 KP 改成 1（PWM = 误差刻度数），让舵机往外走 60 个刻度，看它停在差几个刻度的地方：
剩下的误差就是刚好推不动摩擦的 PWM，换算成力矩。正反两个方向各测 3 次，结束后自动恢复原来的参数。
舵盘要能自由转，最好拆下来单测。

**原版固件**（粗估，不用任何工具）：
```bash
python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 drop left_knee
```
全部卸力，抓住躯干让腿悬空，把这一节扶到水平后松手，看它会不会自己掉下来：
| 关节 | 这一节水平时的重力力矩 |
|---|---|
| hip_roll | 0.154 N·m |
| hip_pitch | 0.094 N·m |
| knee | 0.044 N·m |
| ankle | 0.006 N·m |

停住不动，说明静摩擦大于这个值；慢慢往下滑，说明接近这个值；直接掉下来，说明明显小于它。
