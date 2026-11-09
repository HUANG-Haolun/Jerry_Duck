# 上机用的模型

每个模型都只能配**对应的舵机固件和参数**使用，配错了会直接摔（仿真里已验证：K=8 训练的模型放到原版固件上，0.8 秒就摔倒）。

| 文件 | 舵机固件 | 舵机参数 | 适合 |
|---|---|---|---|
| `jduck_O1_orig_full.onnx` | **原版**（`firmware/stock/send_rev_230400_ult.ino`，朋友烧的那版） | 不用设，固件写死 | 固件改不了的时候 |
| `jduck_N1_k8_calm0.3_caps5.onnx` | **修改版**（`firmware/ubt12hc_pd/`） | **KP=8 FF=0 DB=0**，上机前先设 | 能烧修改版固件的时候 |

两个模型都用**旧 HOME**（hip_pitch ±0.25、knee ∓0.30、ankle ±0.05），HOME 存在 ONNX 元数据里，`jduck_run.py` 会自己读。
都是 61 维观测、14 维动作、50 Hz，大小 776 KB。

## 怎么跑

```bash
# 原版固件
python3 jduck_tools/jduck_run.py --port /dev/ttyS3 --policy models/jduck_O1_orig_full.onnx \
    --imu-config jduck_tools/imu.yaml --dry-run

# 修改版固件：先把 14 个舵机设成 K=8，存进 EEPROM
python3 jduck_tools/servo_params.py --port /dev/ttyS3 set --all --kp 8 --ff 0 --db 0 --save
python3 jduck_tools/jduck_run.py --port /dev/ttyS3 --policy models/jduck_N1_k8_calm0.3_caps5.onnx \
    --imu-config jduck_tools/imu.yaml --dry-run
```
`--dry-run` 只读传感器、跑策略，不给舵机上电。确认关节读数、姿态都正常后再去掉，**第一次上电要把机器人吊起来**。
上机前的完整检查（标定零位、IMU 方向）见 [`jduck_tools/README.md`](../jduck_tools/README.md)。

在电脑上先用键盘玩（MuJoCo，观测和真机一样：ADC 量化、晚一拍、差分速度）：
```bash
cd third_party/microduck_rl
uv run --frozen scripts/infer_policy.py --robot jduck --walking ../../models/jduck_O1_orig_full.onnx \
    --jduck-fw orig --jduck-torque 0.96 --jduck-noload 4.8
uv run --frozen scripts/infer_policy.py --robot jduck --walking ../../models/jduck_N1_k8_calm0.3_caps5.onnx \
    --jduck-fw 8,0,0 --jduck-torque 0.96 --jduck-noload 4.8
```

## 测评结果（仿真，`jduck_tools/hw_chain_eval.py`：和 `jduck_run.py` 一样的控制循环）

15 秒，3 个随机种子取平均。“理想”是指令 × 15 秒。

| 项目 | O1（原版固件） | N1（修改版 K=8） |
|---|---|---|
| 前进 0.2 m/s（理想 +3.0 m） | +2.01 m，偏航 +9° | +2.44 m，偏航 +18° |
| 后退 0.2 m/s（理想 −3.0 m） | −1.45 m，偏航 +5° | −0.86 m，偏航 −112° ⚠ 会自转 |
| 侧走 0.12 m/s（理想 +1.8 m） | +0.94 m | +0.55 m |
| 原地转 0.8 rad/s（理想 +688°） | +540° | +735° |
| 零速：目标每步变化 | 1.23°（关节摩擦 0.08 时 0.58°） | 0.33° |
| 零速：腿关节速度 | 0.12 rad/s | 0.010 rad/s |
| 训练末期摔倒率（参考 r9 0.25） | 0.14 | 0.25 |

- **O1** 各个方向都走得直，但站着会轻微抖。原版固件有 ±2 刻度死区、出死区就是 48 PWM，在目标附近只能一跳一跳地出力；
  仿真里关节摩擦越大越安静，真机齿轮箱的摩擦应该比仿真默认值（0.01 N·m）大，可能会比这里好。
- **N1** 站得更稳，但后退会自转。
- 两个都还**没在真机上跑过**。

## 训练配置（复现用）

在 `third_party/microduck_rl`（`jduck` 分支）下，4096 个环境、4000 次迭代、seed 2：

```bash
# O1
JDUCK_FW=orig JDUCK_OBS_ADC=1 JDUCK_OBS_LAG=1 JDUCK_HOME=old JDUCK_STAND_QUIET=4 JDUCK_STAND_QUIET_S2=1 \
JDUCK_ANG_EMA=0.5 JDUCK_KP_DR=0.85,1.15 JDUCK_TORQUE=0.96 JDUCK_NOLOAD=4.8
# 实验目录 jduck_velocity_ubt12hc_k1ff0db2fl48_t0.96_w4.8_homeold_stand2_quiet4s1_angema0.5_adcobs_lag1_kpdr0.85-1.15/2026-11-09_10-28-18_O1_orig_full

# N1
JDUCK_FW=8,0,0 JDUCK_OBS_ADC=1 JDUCK_OBS_LAG=1 JDUCK_HOME=old JDUCK_STAND_QUIET=4 JDUCK_STAND_QUIET_S2=1 \
JDUCK_ANG_EMA=0.5 JDUCK_KP_DR=0.75,1.25 JDUCK_CAPS_S=5 JDUCK_CAPS_T=0.5 JDUCK_CAPS_WALK=0 JDUCK_CAPS_CALM=0.3 \
JDUCK_TORQUE=0.96 JDUCK_NOLOAD=4.8
# 实验目录 jduck_velocity_ubt12hc_k8ff0db0_..._capsS5T0.5w0c0.3/2026-11-06_21-51-27_N1_k8_calm0.3_caps5（在台式机上）
```
每个变量的含义见仓库根目录 [`README.md`](../README.md) 的训练一节。

| 文件 | sha256 前 16 位 |
|---|---|
| `jduck_O1_orig_full.onnx` | `7420dbe752b462ee` |
| `jduck_N1_k8_calm0.3_caps5.onnx` | `819f5c276344e9ec` |
