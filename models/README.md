# 上机用的模型

每个模型都只能配**对应的舵机固件和参数**使用，配错了会直接摔（仿真里已验证：K=8 训练的模型放到原版固件上，0.8 秒就摔倒）。

| 文件 | 舵机固件 | 舵机参数 | 适合 |
|---|---|---|---|
| `jduck_O1_orig_full.onnx` | **原版**（`firmware/stock/send_rev_230400_ult.ino`，朋友烧的那版） | 不用设，固件写死 | 固件改不了的时候 |
| `jduck_N1_k8_calm0.3_caps5.onnx` | **修改版**（`firmware/ubt12hc_pd/`） | **KP=8 FF=0 DB=0**，上机前先设 | 能烧修改版固件的时候 |

| `jduck_velstand_d1.onnx` | **修改版**，KP=8 FF=0 DB=0 | 同 N1 | **推荐**：一个策略同时会走、会站、摔倒自己爬起来（VelStand） |
| `jduck_getup_jsu14.onnx` | **修改版**，KP=8 FF=0 DB=0 | 同 N1 | **摔倒后自己爬起来**，配 N1 用（见下） |

两个走路模型和起身模型都用**旧 HOME**（hip_pitch ±0.25、knee ∓0.30、ankle ±0.05），HOME 存在 ONNX 元数据里，`jduck_run.py` 会自己读。
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
走路 + 摔倒自己爬起来，单个策略（推荐）：
```bash
python3 jduck_tools/jduck_run.py --port /dev/ttyS3 --policy models/jduck_velstand_d1.onnx \
    --imu-config jduck_tools/imu.yaml --max-tilt 180 --dry-run
```
`--max-tilt 180` 关掉倾斜卸力保护（否则一摔就卸力，没机会爬起来）。

或者用两个策略切换（N1 + 起身模型，`--getup-policy`，效果和上面差不多，留作对照）：
```bash
python3 jduck_tools/jduck_run.py --port /dev/ttyS3 --policy models/jduck_N1_k8_calm0.3_caps5.onnx \
    --getup-policy models/jduck_getup_jsu14.onnx --imu-config jduck_tools/imu.yaml --dry-run
```
倾斜超过 45° 就切到起身模型（此时速度指令被忽略），倾斜 < 12° 且腿关节都回到 HOME 附近（0.35 rad 以内）
保持 0.6 s 后切回 N1。给了 `--getup-policy` 时 `--max-tilt` 卸力保护不再生效（摔倒正是要它自己起来），
**第一次请在软垫上、手扶着试。**

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

## 会走会爬起来的单策略 `jduck_velstand_d1.onnx`（2026-11-10）

用 `scripts/getup/distill_switch.py` 把“N1 走路 + 起身模型 + 切换规则”蒸馏成一个策略（DAgger，在 VelStand 训练环境里，
带随机推倒、各种躺姿出生、舵机随机化）。直接用 PPO 练 VelStand（microduck 的配方）练了 7 轮都不行：
起身动作对 0.03 rad 的误差就很敏感，PPO + BC 只能模仿到 ~0.08 rad。

| 项目（真机控制链路仿真，修改版固件 K=8） | `jduck_velstand_d1` | N1 |
|---|---|---|
| 趴着 / 仰着 / 侧躺起身 | 4/4、4/4、4/4（0.7 / 1.1 / 0.7 s） | 起不来 |
| 走 0.15 m/s，每 6 s 推一下，30 s | 摔 12 次爬起 12 次，前进 +2.00 m | — |
| 零速：目标每步变化 / 腿速度 | 0.324° / 0.0065 rad/s | 0.324° / 0.0102 |
| 零速 K=6 / K=10 | 0.335° / 0.351° | — |
| 前进 0.2 m/s（理想 +3.0 m） | +2.34 m，偏航 +33° | +2.43 m，偏航 +18° |
| 后退 0.2 m/s | −0.76 m，偏航 −116° ⚠ 会自转（同 N1） | −0.88 m，偏航 −112° |
| 原地转 0.8 rad/s（理想 +688°） | +744° | +734° |

训练环境里（`getup_eval.py`，128 个环境、带随机化）：站 100%、蹲 100%、趴 100%、仰 100%、侧躺 99%。

键盘玩（按 **P** 随机推一把，1 m/s，会推倒；方向键走，空格停，q 退出）：
```bash
cd third_party/microduck_rl
uv run --frozen scripts/infer_policy.py --robot jduck --walking ../../models/jduck_velstand_d1.onnx \
    --jduck-fw 8,0,0 --jduck-torque 0.96 --jduck-noload 4.8 --scene ../../jduck_mjcf/scene_groundcontact_view.xml
```
（一定要带 `--scene ...groundcontact...`：默认的走路场景只有脚能碰地，摔倒后身体会穿进地板。）

## 起身模型 `jduck_getup_jsu14.onnx`（2026-11-10）

仿真测评（`jduck_tools/fall_chain_eval.py`：和 `jduck_run.py` 一样的控制循环 + 躯干/头/腿也能碰地的模型，修改版固件 K=8）：

| 项目 | N1 + 起身模型（摔倒后切换） |
|---|---|
| 趴着起身 | 4/4，中位 0.7 s |
| 仰着起身 | 4/4，中位 1.1 s |
| 侧躺起身 | 4/4，中位 0.7 s |
| 走 0.15 m/s，每 6 s 推一下（0.9 m/s），30 s | 推倒 12 次，自己爬起 12 次，最后都站着 |

动作：先收腿，拿头当支点撑一下，再把身体推起来（视频 `~/jduck_ops/jsu14_getup.mp4`）。
起身很吃力矩：舵机只有 0.77 N·m（标称的 80%）时仿真里找不到能起来的动作，真机电压低/舵机热了可能起不来。
头会撞地，**头壳和脖子要结实**。训练过程见 `third_party/microduck_rl` 的 `jduck_standup_env_cfg.py` 和 CLAUDE.md。

```bash
cd third_party/microduck_rl
uv run --frozen python ../../jduck_tools/fall_chain_eval.py ../../models/jduck_N1_k8_calm0.3_caps5.onnx \
    --getup ../../models/jduck_getup_jsu14.onnx
```

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
