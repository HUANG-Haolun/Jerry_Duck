# 上机用的模型

每个模型都只能配**对应的舵机固件和参数**使用，配错了会直接摔（仿真里已验证：K=8 训练的模型放到原版固件上，0.8 秒就摔倒）。

| 文件 | 舵机固件 | 舵机参数 | 适合 |
|---|---|---|---|
| `jduck_O21b_orig.onnx` | **原版** | 不用设；**必须加 `--stand-hold`** | **推荐（2026-11-12）**：只管走路，零速由程序保持 HOME；各方向走得直，见下 |
| `jduck_O23a_orig.onnx` | **原版** | `--stand-hold --wz-scale 0.8` | 和 O21b 同配方、另一个种子：1.25/1.4 N·m 全过，1.55 下后退偏航 −34°（O21b 正好相反），见下 |
| `jduck_O18a_orig.onnx` | **原版** | 加 `--stand-hold` | O21b 之前的最好版本，1.55 N·m 下偏航稍大 |
| `jduck_O8d_orig.onnx` | **原版**（朋友烧的那版） | 不用设 | 2026-11-11 的推荐；真机会往后倒，已被 O21b 取代 |
| `jduck_O1_orig_full.onnx` | **原版**（`firmware/stock/send_rev_230400_ult.ino`，朋友烧的那版） | 不用设，固件写死 | 固件改不了的时候 |
| `jduck_N1_k8_calm0.3_caps5.onnx` | **修改版**（`firmware/ubt12hc_pd/`） | **KP=8 FF=0 DB=0**，上机前先设 | 能烧修改版固件的时候 |

| `jduck_velstand_d1.onnx` | **修改版**，KP=8 FF=0 DB=0 | 同 N1 | **推荐**：一个策略同时会走、会站、摔倒自己爬起来（VelStand） |
| `jduck_kick_right_k8.onnx` | **修改版**，KP=8 FF=0 DB=0 | 同 N1 | **右脚踢球**（站着时切过去，踢完 3 s 切回走路模型） |
| `jduck_getup_jsu14.onnx` | **修改版**，KP=8 FF=0 DB=0 | 同 N1 | **摔倒后自己爬起来**，配 N1 用（见下） |

两个走路模型和起身模型都用**旧 HOME**（hip_pitch ±0.25、knee ∓0.30、ankle ±0.05），HOME 存在 ONNX 元数据里，`jduck_run.py` 会自己读。
都是 61 维观测、14 维动作、50 Hz，大小 776 KB。

## 原版固件推荐：`jduck_O21b_orig.onnx` + `--stand-hold`（2026-11-12）

```bash
cd /root/Jerry_Duck && jduck_tools/run_robot.sh -m models/jduck_O21b_orig.onnx --stand-hold --wz-scale 0.8
```
`--wz-scale 0.8`：发给策略的转向指令是按键的 0.8 倍。O21b 在 0.25 rad/s 下会多转 25–40%，乘 0.8 后左/右转在三种力矩下都在理想值的 ±13% 以内。
松开方向键 0.4 s 后动作在 0.4 s 内收回 HOME 并保持（零速完全不抖），一按方向键立即交回策略。

为什么这样：原版固件 `pwm = clamp(|误差|, 48, 255)` 出了死区就是 48 档，2026-11-12 吊空阶跃测试拟合出
真舵机是 1.25–1.4 N·m（不是规格书的 0.96）、空载 5.5 rad/s、+5 ms 延迟；在这么硬的舵机上，O20a–c 无论怎么加零速奖励
都抖 0.9–1.5°/拍，而直接保持 HOME 一点不抖、真机 `--hold-home` 也站得稳。所以 O21 只练走路
（O12 配方 + 上述舵机参数 + 航向保持 + 左右对称），零速奖励全部去掉。

校准仿真测评（`calib_stand_eval.py --walk --stand-hold`，按键速度，每项 15 s，种子 5–8 复核）：

| 指令（理想） | 力矩 1.25 N·m | 1.4 N·m | 1.55 N·m |
|---|---|---|---|
| 前进 0.08（+1.2 m） | +1.09 m，偏航 +3° | +1.08 m，+2° | +1.01 m，+14° |
| 后退 0.08（−1.2 m） | −0.86 m，−1° | −0.94 m，+15° | −0.97 m，+17° |
| 左移 0.05（+0.8 m） | +0.75 m，−9° | +0.76 m，−15° | +0.78 m，−15° |
| 右移 0.05（−0.8 m） | −0.52 m，−31° | −0.49 m，−7° | −0.49 m，+13° |
| 左转 0.25（+215°） | +266° | +265° | +255° |
| 右转 0.25（−215°） | **−300°** | **−288°** | −267° |
| 走 4 s 后停下 | 6 个方向全站住 | 全站住 | 全站住 |

- 不缩放转向时：1.55 N·m 下过了全部门槛；1.25/1.4 下只有**右转多转 34–40%**（门槛 30%）。
- **加 `--wz-scale 0.8` 后（种子 5–8）**：左转 +217° / +216° / +206°，右转 −244° / −239° / −223°（理想 ±215°，三种力矩依次）；
  **1.4 和 1.55 N·m 下全部门槛通过**，1.25 下只剩右移偏航 −31°（门槛 30°）。
- 上电 1 s 渐入 + 零速保持：静止 3 s 后起步前进 / 后退 / 左转 / 右转，三种力矩 × 4 个种子全站住，最大倾斜 2–8°。

**O23a**（同配方，训练种子 5；测评种子 5–8，`--wz-scale 0.8`）：

| 指令 | 1.25 N·m | 1.4 N·m | 1.55 N·m |
|---|---|---|---|
| 前进 / 后退 0.08 偏航 | −25° / −5° | −19° / −25° | −14° / **−34°** |
| 左移 / 右移 0.05 偏航 | −12° / −23° | −9° / −25° | −6° / −25° |
| 左转 / 右转 0.25（理想 ±215°） | +242° / −243° | +243° / −239° | +224° / −234° |
| 停下 / 起步 | 全站住 | 全站住 | 全站住 |

真舵机 1.25–1.4 N·m（阶跃拟合）落在 O23a 全过的区间；电池满电舵机偏硬时用 O21b。两个都先试，哪个真机走得直用哪个。
- 头的打印件必须装牢：头+脖子占 36% 质量，头松了走路会往后仰（2026-11-12 的后仰录制就是头掉了的时候录的）。
- `imu_spi.yaml` 的 `pitch_offset_deg` 先设 0：1.8° 是头掉了时测的，头装好后要用 `--hold-home` 重测。

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

## 原版固件推荐模型 `jduck_O8d_orig.onnx`（2026-11-11）

**上真机：**
```bash
cd /root/Jerry_Duck
jduck_tools/run_robot.sh -m models/jduck_O8d_orig.onnx
```
> ⚠ 2026-11-11 真机上 O8d 会往前倒（站姿弯膝、髋 pitch 不对称，是按偏高的 1.2 N·m 力矩练出来的）。先用 O1，等 O11。
键盘：方向键走，a / e 转，空格停，q 退出。按键速度上限保持默认（vx 0.08、vy 0.05、wz 0.25）：
**这个模型就是按这个速度范围练的**，按更快的速度（0.2 m/s）会走歪。

**测评（`jduck_tools/calib_stand_eval.py --walk`：用真机零速数据校准过舵机参数的仿真，15 s，4 个随机种子）：**

| 项目 | 标准 | O8d | O1 |
|---|---|---|---|
| 零速：腿每拍目标变化 | < 0.5° | **0.46°** | 2.26° |
| 零速：腿关节实际抖动 | — | 0.36° | 1.20° |
| 前进 0.08 m/s（理想 +1.2 m） | ≥ 0.6 m，偏航 < 30° | +1.02 m，偏航 +27° | +0.75 m，偏航 −70° |
| 后退 0.08 m/s（理想 −1.2 m） | ≥ 0.6 m，偏航 < 30° | −1.05 m，偏航 −29° | −0.02 m（不动） |
| 左移 0.05 m/s（理想 +0.8 m） | ≥ 0.4 m，偏航 < 30° | +0.79 m，偏航 −9° | +0.47 m，偏航 −41° |
| 右移 0.05 m/s（理想 −0.8 m） | ≥ 0.4 m，偏航 < 30° | −0.71 m，偏航 +20° | −0.22 m，偏航 −68° |
| 左转 0.25 rad/s（理想 +215°） | 误差 ±30 % | +200° | +212° |
| 右转 0.25 rad/s（理想 −215°） | 误差 ±30 % | −242° | −317° |

在原默认仿真（`hw_chain_eval.py --fw orig`）里零速每拍目标变化 0.28°（O1 1.21°）。前进、后退的偏航离 30° 的线很近，真机上可能会慢慢偏，需要时用 a / e 修。

**怎么练出来的**（`scripts/longrun/long_O.sh` 的 O8d，seed 3，5000 次迭代，台式机约 75 分钟）：
```bash
JDUCK_FW=orig JDUCK_OBS_ADC=1 JDUCK_OBS_LAG=1 JDUCK_HOME=old JDUCK_STAND_QUIET=4 JDUCK_STAND_QUIET_S2=1 JDUCK_ANG_EMA=0.5 \
JDUCK_KP_DR=0.85,1.15 JDUCK_TORQUE=1.2 JDUCK_NOLOAD=4.8 JDUCK_TORQUE_DR=0.95,1.3 JDUCK_FRICTION=0.005 JDUCK_FRICTION_DR=0,2 \
JDUCK_ACT_DELAY=2,6 JDUCK_GYRO_NOISE=0.15 \
JDUCK_CAPS_S=10 JDUCK_CAPS_T=1 JDUCK_CAPS_WALK=0 JDUCK_CAPS_CALM=0.3 \
JDUCK_CMD=0.12,0.08,0.4 JDUCK_LIN_STD=0.06 JDUCK_ANG_STD=0.12 JDUCK_CMD_CURR=1500 JDUCK_LIN_REL_W=2 \
uv run --frozen train Mjlab-Velocity-Flat-JDuck --env.scene.num-envs 4096 --agent.max_iterations 5000 --agent.seed 3
```
和 O1 比，改了这几样（每一样都是前一轮测出来的问题）：
1. **舵机参数按真机校准**（`real_calib.py`）：力矩 1.2 N·m（随机 ×0.95～1.3）、关节摩擦接近 0、指令延迟 10～30 ms、陀螺噪声 ±0.15。
2. **零速平滑**（CAPS，`JDUCK_CAPS_*`）：只在零速时约束策略对传感器噪声不敏感。
3. **速度范围先宽后窄**（`JDUCK_CMD_CURR=1500`）：前 1500 次用 ±0.25 m/s 学会迈步，之后缩到按键范围 ±0.12 / ±0.08 / ±0.4。
   一开始就用窄范围的话学不会迈步（O5）。
4. **按相对误差计分的速度奖励**（`JDUCK_LIN_REL_W`）：原来的速度奖励在 0.08 m/s 时站着不动也能拿 85 %，
   所以以前的模型按键时只是原地挪；只把宽度收窄又会失去梯度（O7）。新奖励看“走到了指令的几成”，站着不动 0.02、走到位 1.0。
5. **偏航跟踪收紧**（`JDUCK_ANG_STD=0.12`）：原来 15 s 偏 75° 还能拿 91 % 的转向奖励。

完整过程（O3～O9，每一轮哪项没过）见 `~/jduck_ops/long_O/summary.txt`。

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

## 右脚踢球 `jduck_kick_right_k8.onnx`（2026-11-10）

`Mjlab-BallKick-Flat-JDuck`（microduck 原版踢球配方 + jduck 的脚位和舵机），纯 PPO 5000 次迭代，约 100 次迭代就学会了。
策略**看不见球**：球要摆在右脚尖前约 2 cm（球心在躯干前 9 cm、右侧 6.4 cm），偏差 ±1.5 cm 以内都能踢到。
球是 70 mm / 15 g 的空心塑料球（类似地板球）。

训练环境里测（`scripts/getup/kick_eval.py`，256 次，推力关闭）：

| 项目 | 结果 |
|---|---|
| 踢到球 | 100%，开始后 0.12 s 就踢 |
| 最大球速 | 中位 1.51 m/s（训练目标 1.0，踢得偏猛） |
| 球滚出去的距离（5 s 内） | 中位 5.4 m |
| 方向偏差 | 中位 17°（偏向一侧，不是正前方） |
| 摔倒 | 0% |
| 支撑脚离地 | 平均 0.03 s（基本不离地） |

真机控制链路仿真里不放球、零速站 15 s：K=8 时每步目标变化 0.24°，不摔；**K=10 时会抖（1.06°）**，舵机 K 一定要设成 8（或 6）。

键盘玩（终端里按 **L** 放球并踢，3 s 后切回走路模型）：
```bash
cd third_party/microduck_rl
uv run --frozen scripts/infer_policy.py --robot jduck --walking ../../models/jduck_velstand_d1.onnx \
    --kick-right ../../models/jduck_kick_right_k8.onnx --jduck-fw 8,0,0
```
`jduck_run.py` 还不支持按键切到踢球模型，上真机要先补这个功能。

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
| `jduck_O8d_orig.onnx` | `9e79d28fe0273813` |
| `jduck_O1_orig_full.onnx` | `7420dbe752b462ee` |
| `jduck_N1_k8_calm0.3_caps5.onnx` | `819f5c276344e9ec` |
| `jduck_N3k4a_kp4.onnx` | `1a0035bc5602748c` |
| `jduck_N5k4a_kp4.onnx` | `f0816397a97065f9` |
| `jduck_N6home3k4a_kp4.onnx` | `b8953cc8cae298f4` |

## KP=4 固件：N3k4a（2026-11-16）

`jduck_N3k4a_kp4.onnx` 是当前候选策略，按 `scripts/longrun/n3_train.sh` 在台式机训练。
校准仿真在堵转力矩 1.25 / 1.4 / 1.55 N·m 下，零速腿目标变化约 0.25–0.28°/拍；
直接运行策略时站姿最大偏离 HOME 约 9°。使用 `--stand-hold --stand-ankle-deg 3` 后，
三个力矩档的零速站姿俯仰分别为 +0.9° / +0.2° / −0.5°，六种走停切换均站住。
KP=4 固件和该站姿组合尚未真机验证。上机命令见 [`jduck_tools/README.md`](../jduck_tools/README.md)。
在有图形桌面的终端交互仿真：
```bash
cd third_party/microduck_rl
uv run --frozen scripts/infer_policy.py --robot jduck --walking ../../models/jduck_N3k4a_kp4.onnx \
  --jduck-fw 4,0,0 --jduck-torque 1.4 --jduck-noload 5.5 \
  --jduck-keys 0.08,0.05,0.25 --stand-hold --stand-ankle-deg 3
```

N4k4a 只额外增加权重 4 的十关节 HOME L1 惩罚，训练最终回合长度约 551（N3 约 983），
第 3999 轮在校准仿真中零速 4/4 摔倒。第 2000 和 2500 轮 checkpoint 也未通过零速与走路门槛，
故 N4 不作为候选模型。

N5k4a 在 N3 基础上从第 1000 轮开始加入较弱的 HOME L1 惩罚（权重 0.2）。
在 1.25 / 1.4 / 1.55 N·m 校准仿真中，配合 `--stand-hold --stand-ankle-deg 3`，
静站躯干俯仰为 +1.1° / +0.2° / −0.5°，各档六种走停切换均站住。
1.4 N·m 下低速前进 15 s 偏航 −9°，但左移偏航 −28°，0.2 m/s 前进偏航 −28°；
1.55 N·m 下 0.2 m/s 后退偏航 +31°。可用于仿真，尚未真机验证。

```bash
cd third_party/microduck_rl
uv run --frozen scripts/infer_policy.py --robot jduck --walking ../../models/jduck_N5k4a_kp4.onnx \
  --jduck-fw 4,0,0 --jduck-torque 1.4 --jduck-noload 5.5 \
  --jduck-keys 0.08,0.05,0.25 --stand-hold --stand-ankle-deg 3
```

N6home3k4a 是新的 HOME 对照实验：沿用 N3 的其余训练参数，
把 HOME 的左右踝从 ±0.05 rad 改为 ±(0.05 rad + 3°)，并按足底高度把初始躯干高度增加 1 mm。
它从头训练，不能把此 HOME 直接套到 N3 的 ONNX 上。

N6 已完成 4000 轮训练和校准仿真（2026-11-16）。配合 `--stand-hold`、无需额外踝偏置时，
1.25 / 1.4 / 1.55 N·m 的静站躯干俯仰为 +2.0° / +0.2° / −0.5°，六种走停切换均站住。
1.4 N·m 下前进 0.2 m/s 行走 15 s 的偏航约 −4°（N3 约 −32°），
但后退 0.2 m/s 偏航约 +84°、侧移时前后漂移约 0.24 m，仍不适合替换 N3 上机。
N6 只用于仿真对照；高速度后退是下一步需要解决的问题。

```bash
cd third_party/microduck_rl
uv run --frozen scripts/infer_policy.py --robot jduck --walking ../../models/jduck_N6home3k4a_kp4.onnx \
  --jduck-fw 4,0,0 --jduck-torque 1.4 --jduck-noload 5.5 \
  --jduck-keys 0.08,0.05,0.25 --stand-hold
```
