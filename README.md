# Jerry Duck（jduck）

一只 Microduck 风格的小型双足机器人，用强化学习走路。总重 1.37 kg，14 个 UBT-12HC 总线舵机：
每条腿 5 个，脖子和头 4 个，另外嘴巴 1 个是被动关节。

整体流程：SolidWorks 模型 → URDF → MuJoCo 模型 → 在 [mjlab](https://github.com/mujocolab/mjlab)
（MuJoCo Warp + PPO）里训练 → 导出 ONNX → 在泰山派 RK3566 上以 50 Hz 跑策略，通过总线驱动舵机。
训练代码改自 Pollen Robotics 的 [microduck_rl](https://github.com/pollen-robotics/microduck_rl)。

> **状态（2026-11）**：已经能在仿真里走、转弯，零速时能站住。完整的真机控制程序也在仿真里跑通了。
> **还没有在真机上跑过**：关节标定、IMU 安装方向、修改版舵机固件，都还要在实物上验证。

## 目录

```
rl_control/
├── jduck_description/   机器人模型（ROS 2 包）：URDF、STL、RViz 启动文件
├── jduck_mjcf/          MuJoCo 模型，由 build_mjcf.py 从 URDF 生成（不要手改 XML）
├── models/              上机用的 ONNX 模型和说明（哪个模型配哪个固件、测评结果）
├── jduck_tools/         真机端：舵机协议、关节标定、IMU、50 Hz 控制程序、仿真替身
├── firmware/
│   ├── ubt12hc_pd/      修改版舵机固件（线性 P 控制律、参数可在线调、通信超时）
│   └── stock/           朋友烧的原版固件，只作参考，不要改
├── scripts/             独立小工具：URDF 查看器、控制律仿真筛选
├── ros2_ws/             colcon 工作区，src/ 里是指向上面 ROS 包的软链接
└── third_party/         git 子模块
    ├── microduck_rl/    训练仓库（jduck 相关的都在 jduck 分支上）
    └── microduck/       Pollen 原版的机器人端运行时（Rust），只作参考
```

## 快速开始

```bash
git clone --recurse-submodules https://github.com/HUANG-Haolun/Jerry_Duck.git rl_control
cd rl_control/third_party/microduck_rl && git checkout jduck
```

训练环境用 [uv](https://docs.astral.sh/uv/) 管理，需要 CUDA GPU（已在 RTX 5090 上测过）。
国内网络建议用镜像：`uv pip install --index-url https://mirrors.aliyun.com/pypi/simple/ <包名>`。
后面训练相关的命令都在 `third_party/microduck_rl/` 下执行，并加上 `--frozen`，避免 uv 重新解析依赖。

### 在仿真里用键盘玩

```bash
cd third_party/microduck_rl
uv run --frozen scripts/infer_policy.py --robot jduck --walking ../../models/jduck_O1_orig_full.onnx \
    --jduck-fw orig --jduck-torque 0.96 --jduck-noload 4.8
```
方向键控制前后左右，A / E 原地转，空格停。`--jduck-fw` 必须和模型训练时的舵机固件一致，
每个模型该配什么见 [`models/README.md`](models/README.md)。

### 看模型

```bash
python3 scripts/view_urdf.py jduck_description/urdf/jduck_description.urdf   # PyBullet，不需要 ROS
cd third_party/microduck_rl && uv run --frozen python -m mujoco.viewer --mjcf ../../jduck_mjcf/scene_view.xml

# RViz（系统 ROS 2 Jazzy）
source /opt/ros/jazzy/setup.bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch jduck_description display.launch.py
```

## 训练

训练任务叫 `Mjlab-Velocity-Flat-JDuck`。它在 microduck 的速度跟踪任务基础上换成了 jduck 的模型，
具体配置通过 `JDUCK_*` 环境变量选择，实验名会自动带上这些参数。

当前推荐的配置：
```bash
cd third_party/microduck_rl
JDUCK_OBS_ADC=1 JDUCK_HOME=old JDUCK_FW=16,0,0 JDUCK_TORQUE=0.96 JDUCK_NOLOAD=4.8 \
JDUCK_STAND_QUIET=4 JDUCK_STAND_QUIET_S2=1 JDUCK_ANG_EMA=0.5 \
uv run --frozen train Mjlab-Velocity-Flat-JDuck --env.scene.num-envs 4096 \
    --agent.max_iterations 4000 --agent.seed 1 --agent.run-name mytry --agent.logger tensorboard
```
在 RTX 5090 台式机上大约 50 分钟，笔记本 5090 Laptop 上大约 1.6 小时。正式跑之前，
先用 `--env.scene.num-envs 64 --agent.max_iterations 5` 做一次冒烟测试。

| 变量 | 推荐值 | 作用 |
|---|---|---|
| `JDUCK_HOME` | `old` | 站立姿态。`old` 蹲得更低，走路和站立都明显比 `new` 好 |
| `JDUCK_FW` | `16,0,0` | 舵机控制律 K,FF,DB：`pwm = K·|误差| + FF`，误差 ≤ DB 时为 0。不设时是原版固件的控制律，用它训练学不会迈步 |
| `JDUCK_TORQUE` / `JDUCK_NOLOAD` | `0.96` / `4.8` | 舵机堵转力矩（N·m）和空载转速（rad/s），按官方 7.4 V 规格 |
| `JDUCK_OBS_ADC` | `1` | 关节观测和真机一致：位置按 ADC 量化，速度用 20 ms 差分。不开的话，策略上真机后零速时会抖 |
| `JDUCK_STAND_QUIET` / `_S2` | `4` / `1` | 零指令时“动作越安静越给分”的正奖励，让机器人站着不动 |
| `JDUCK_ANG_EMA` | `0.5` | 转向奖励按 0.5 s 滑动平均的偏航角速度计算。不开的话，基本学不会转弯 |
| `JDUCK_STAND` | `v2`（默认） | 零指令站立奖励的版本：脚着地、不漂移 |
| `JDUCK_STAND_ACT_RATE` | 不用 | 零指令动作变化率惩罚。超过 −5 就会把策略练崩，已经被 `STAND_QUIET` 取代 |

### 导出和测评

```bash
scripts/jduck_export.sh logs/rsl_rl/<实验名>/<日期>_mytry 3999 jduck_mytry.onnx   # 环境变量会从实验名自动还原
uv run --frozen python scripts/jduck_eval.py jduck_mytry.onnx                     # 训练式观测：走、转、站
uv run --frozen python ../../jduck_tools/hw_chain_eval.py jduck_mytry.onnx         # 真机式观测：以这个结果为准
```
`hw_chain_eval.py` 用的是和 `jduck_run.py` 完全一样的控制循环，接在 `sim_robot.py` 这个仿真替身上跑：
关节位置是量化后的 ADC 读数，速度是差分算出来的。`jduck_eval.py` 喂给策略的是理想观测，
之前有好几个策略在它下面完全站得住，换到真机式观测上就会抖，所以**判断能不能上机，要看 `hw_chain_eval.py`**。
零速时“目标每步变化”要在 0.1° 以下。

### 摔倒后自己爬起来（2026-11-10）

起身策略 `Mjlab-StandUp-Flat-JDuck`（`jduck_standup_env_cfg.py`）。jduck 头又长又重（头+脖子占 36% 质量），
直接照搬 microduck 的配方怎么练都起不来，最后是这样做出来的：

1. `scripts/getup/cem_env.py`：在训练环境里用 CEM 搜关键帧动作，每个候选在 32 个带随机化、带动作噪声的环境里打分，
   得到抗扰动的“趴→站”和“仰→趴”两段动作（早先在简化仿真里搜到的动作加 0.03 rad 噪声就全失败）。
2. `scripts/getup/record_env.py`：带噪声回放、录下成功的状态；`scripts/getup/dagger_getup.py`：用“最近录制状态的下一步动作”当专家做 DAgger。
3. 从 DAgger 结果接着跑 PPO（`MICRODUCK_WARM_START=1`，`JDUCK_REPLAY_PROB=0.3` 从录制状态出生，`JDUCK_IMIT_W=2` 模仿奖励），
   1250 次迭代就从趴/仰/侧躺 100% 站起，约 1 s。这就是 `models/jduck_getup_jsu14.onnx`。
   起身对力矩很敏感，所以用 `JDUCK_TORQUE_DR=0.95,1.15 JDUCK_KP_DR=0.9,1.25`。

4. 会走又会爬起来的单策略 `models/jduck_velstand_d1.onnx`：`scripts/getup/distill_switch.py` 用 DAgger 把
   “N1 + 起身模型 + `jduck_run.PolicySwitch` 的切换规则”蒸馏成一个策略（VelStand 环境，10 轮，笔记本约 25 分钟）。
   直接 PPO 练 `Mjlab-VelStand-Flat-JDuck`（microduck 配方，jvs1–7）都不行：动作噪声会涨到 0.4，起身全失败，
   策略学会趴着不动；噪声封顶（`JDUCK_VS_MAX_STD`）、加大 BC（`JDUCK_VS_BC_COEF`）后也只有 20–50%。

上机见 [`models/README.md`](models/README.md)。

### 在另一台机器上训练

`third_party/microduck_rl/scripts/jduck_pipeline.sh <名字> 4000 -- JDUCK_...=...` 会依次完成
训练、导出和测评，结果写到 `logs/pipeline/<名字>.txt`。适合用 ssh 在台式机上挂着跑。

## 上真机

真机端代码全在 `jduck_tools/`，跑在机器人背上的**泰山派**（RK3566，Debian 12）上，不需要 ROS 和 uv。
详细步骤见 [`jduck_tools/README.md`](jduck_tools/README.md)。

**现在的标准流程（2026-11-10）**：

1. **连板子**：USB 转串口接调试串口，`python3 -m serial.tools.miniterm /dev/ttyUSB0 1500000`（Ctrl+] 退出）。
   板子没有网口；连手机热点后能上外网，但和公司电脑不在一个网段，ssh 不了。
2. **准备环境（只做一次）**：板子上 `cd /root/Jerry_Duck && jduck_tools/setup_board.sh`。
   会建 `/userdata/jduck-venv`（离线装 numpy / onnxruntime 等），编译 IMU 读数库，并自检读一次 IMU。
3. **启动**：机器人扶正后 `jduck_tools/run_robot.sh`（默认 O1、原版固件）；`-m` 换模型，`--hold-home` 只保持 HOME，
   `--log` 录数据，`-- --dry-run` 不上电。键盘：方向键走，a / e 转，空格停，q 退出。
4. **新模型先过仿真门槛再上机**：`jduck_tools/calib_stand_eval.py --walk`（用真机数据校准过舵机参数的仿真），
   零速腿每拍目标变化 < 0.5°，按键速度下各方向不摔、偏航 < 30°，后退能走出 0.6 m。

**硬件现状**：
- 舵机是**原版固件**（改不了），只能用 `JDUCK_FW=orig` 训练的模型（O1 系列）。修改版固件的 N1 / d1 / 踢球 / 起身模型上不了机。
- IMU 是 **ICM-42688P（SPI）**，不是 LSM6DSV16X；驱动 `imu_spi_board.py`，配置 `imu_spi.yaml`（轴向、零偏已标定）。
- `--hold-home` 看真机 HOME 和仿真一致；零位暂时不用重标。
- 实测（2026-11-10）：控制周期中位 20 ms，每个舵机一问一答 1.29 ms，推理加读 IMU 3.6 ms。
  O1 零速时腿每拍目标变化 2.8°（仿真里 1.2°），整机有 4 Hz 左右的抖动。按真机数据校准仿真后（舵机更有力、关节更滑、约多 10 ms 延迟）
  重训的 O2cal 能减少约 1/4，但还在抖，后退还会摔。台式机和笔记本正在按 `scripts/longrun/long_O.sh` 继续练，直到过门槛。

**烧修改版固件（如果以后能改）**：先拿一个舵机试 `firmware/ubt12hc_pd`，见 [`firmware/ubt12hc_pd/README.md`](firmware/ubt12hc_pd/README.md)；
参数用 `jduck_tools/servo_params.py` 改，KP / FF / DB 要和训练时的 `JDUCK_FW` 一致。

## 重要约定

- **改了 URDF 就要重新生成 MJCF**：`cd third_party/microduck_rl && uv run --frozen python ../../jduck_mjcf/build_mjcf.py`。
  MJCF 用的是 microduck 的命名（`trunk_base`、不带 `_joint` 后缀、`passive_mouth`），这样才能复用它的训练配置。
- **jduck 的 URDF 是 CRLF 换行**。用 Python 改的时候要加 `newline=''`，否则 diff 会显示每一行都改了。
- **关节顺序和 61 维观测布局**沿用 microduck，见 `third_party/microduck_rl/AGENTS.md`：
  左腿 5 个、脖子和头 4 个、右腿 5 个；观测是 48 维本体感知加 13 维指令。
- **舵机参数改了要同步到仿真**：固件的 KP / FF / DB 和训练时的 `JDUCK_FW` 必须一致。
- `jduck_description/urdf/*.bak` 是手动修改前的备份：`.urdf.bak` 是改惯量之前的版本，`.urdf.precal.bak` 是写入标定范围之前的版本。
  从 SolidWorks 重新导出会覆盖 `.urdf`，导出后要重新修一遍。
- ROS 用系统的 **ROS 2 Jazzy**（`/opt/ros/jazzy`）。

## 已知问题

- 边走边转时，几乎不往前走；侧走也偏弱。
- 原地转弯会转得比指令多 10%–20%。
- `hip_yaw` 往一侧只有 26° 的转动范围，转弯时经常顶到限位。这个范围可能是零位标错导致的，要在真机上确认。
- 关节力矩和速度（URDF 里 0.8 N·m / 3.0 rad/s，训练里 0.96 N·m / 4.8 rad/s）以及摩擦，都还没有在实物上测过。
