# 传统步态实验程序

本目录目前是传统步态的**硬件调试原型**，用于分步验证舵机通信、初始姿态、重心转移和 IMU 姿态反馈。程序面向仓库中的 RK3566/Linux 硬件，使用 `/dev/ttyS3`、230400 baud；嘴部舵机暂不控制。

## 程序

| 文件 | 用途 |
|---|---|
| `return_init.c` | 读取 `jduck_tools/calibration.yaml` 中 14 个舵机的 `adc_zero`，读取当前位置并平滑回初始 ADC 姿态。默认 3 秒。 |
| `gait_step.c` | 命令行逐次切换四个预设阶段：双脚支撑、左支撑右摆、双脚支撑、右支撑左摆。阶段编号保存在仓库根目录 `.gait_phase`。当前是关节偏移示例，不是足端轨迹步态。 |
| `gait_probe.c` | 单次执行 `stand`、`shift_left`、`shift_right`、`lift_left`、`lift_right` 或 `home`，用于人工检查关节方向和动作幅度。 |
| `probe_cycle.sh` | 顺序调用 `gait_probe`，执行站立、左右重心转移、抬腿和回初始姿态的调试流程。默认抬腿偏移为 0 度。 |
| `gait_imu_pd.c` | ICM42688P 姿态读取及 PD 补偿原型。当前未与四阶段步态整合，IMU 轴映射、偏置和补偿方向尚需验证。 |
| `jduck_tools/traditional_gait_sim.py` | 使用现有 MuJoCo `SimRobot` 仿真站姿和重心转移；摆动腿可按足端目标轨迹用 MuJoCo 雅可比做数值 IK。不打开真机串口，可输出 CSV。 |
| `jduck_tools/sweep_traditional_gait.py` | 网格扫描重心偏移、步长和步高，按最大躯干倾角及足端离地量筛选候选参数。 |
| `jduck_tools/optimize_weight_shift.py` | 随机搜索髋 Roll/Pitch、膝、踝组合，评估质心、足端高度和躯干倾角。 |
| `jduck_tools/traditional_gait_walk_sim.py` | 连续 50 Hz 关节空间交替摆动试验，带可选躯干 Roll/Pitch PD；输出存活时间、前进位移、倾角和脚底高度。 |

## 编译

在仓库根目录执行：

```bash
gcc -O2 -Wall scripts/traditional_gait/return_init.c -lm -o return_init
gcc -O2 -Wall scripts/traditional_gait/gait_step.c -lm -o gait_step
gcc -O2 -Wall scripts/traditional_gait/gait_probe.c -lm -o gait_probe
gcc -O2 -Wall scripts/traditional_gait/gait_imu_pd.c jduck_tools/icm42688/spi_adapter.c -lm -o gait_imu_pd
```

仿真入口使用 `third_party/microduck_rl` 的 Python 环境：

```bash
cd third_party/microduck_rl
uv run --frozen python ../../jduck_tools/traditional_gait_sim.py --action cycle --shift-deg 3 --step-length 0.01 --step-height 0.01 --csv /tmp/gait.csv
uv run --frozen python ../../jduck_tools/sweep_traditional_gait.py --shift 0,2,4,6 --length 0,0.002,0.004 --height 0,0.001,0.002,0.003 --move 0.6
uv run --frozen python ../../jduck_tools/optimize_weight_shift.py --trials 120 --seconds 0.5 --support left
uv run --frozen python ../../jduck_tools/traditional_gait_walk_sim.py --seconds 10 --period 1.2 --roll 3 --hip 4 --knee 12 --ankle -3 --balance-kp 0.4 --balance-kd 0.05 --swing-ratio 0.3
```

可将 `--scene groundcontact` 用于启用躯干地面碰撞的场景；默认 `walk` 场景与现有仿真控制一致。单独仿真动作可将 `--action` 设为 `stand`、`shift_left`、`shift_right`、`lift_left` 或 `lift_right`。抬脚参数 `--step-length`、`--step-height` 单位为米。

当前数值 IK 尚未通过稳定性验证：本地 48 组步态参数扫描没有找到能完成周期的稳定组合；单靠髋 Roll 移重心时，支撑脚会明显离地。之后又对 120 组髋 Roll/Pitch、膝、踝随机姿态做了仿真搜索，未找到同时满足支撑脚离地小于 3 mm、摆动脚离地约 8 mm、COM 位于支撑脚范围且躯干倾角小于 12° 的候选姿态。最佳候选仍离地约 9 mm。因此当前模型下还不能声称存在稳定的单脚支撑参数，也不能据此上真机；需要检查姿态几何、接触模型和控制目标，再继续搜索。

连续步态脚本现可设置舵机堵转扭矩、空载转速、关节/地面摩擦、启动渐入、摆动前重心预移、初始姿态/角速度扰动，并统计腾空比例和支撑切换次数。按规格换算 `stall=1.471 N·m / no_load=4.55 rad/s`，基准候选为 `period=2.0s / roll=1° / roll_bias=7° / hip=-4°（direction=-1）/ knee=22° / ankle=-9° / Kp=0.35 / Kd=0.05 / swing_ratio=0.30 / startup=1.5s / fw_k=20`。基准仿真 120 秒存活，最大倾角 17.25°，左右脚腾空比例 12–13%，支撑切换 513 次，前进 0.891 m（约 7.4 mm/s）。扭矩降至 1.35 N·m、空载速度降至 4.3 rad/s 时也存活 60 秒，但速度降至约 4.9 mm/s；1.2 N·m / 4.0 rad/s 时虽未摔倒，腾空比例仅 5%，几乎变成拖步。Kp 增至 0.45 时在 34.4 秒倾倒，说明控制增益敏感。初始躯干分别施加 ±2° Roll 或 ±2° Pitch，四组均存活 20 秒；2秒周期下四种 Roll/Pitch 组合扰动（±2°/±2°）均存活 60 秒。初始偏置下把关节摩擦从默认 0.02 N·m 改为 0.01 和 0.04 N·m，两组均存活 40 秒。地面摩擦 0.5、0.75、1.25、1.5 下施加组合初始偏置后均存活 40 秒；摩擦 1.5 时腾空仅约 1%–4%，几乎退化为拖步；该场景扫摆动比例和髋幅也未找到有效稳定组合。提速尝试：周期 1.7 秒、预移 5°、Kp=0.35 时，四种组合初始扰动中的三种存活 60 秒（前进 0.22–0.28 m，腾空约 11%–14%），但剩余一象限会倾倒。改为预移 7°、Kp=0.4 后，虽然 `Roll=+2°/Pitch=+2°` 可通过 30 秒，四种组合扰动的 60 秒复测全部倾倒（约 11–38 秒）。2秒候选增加初始角速度 ±10°/s Roll、±10°/s Pitch 及组合（Roll +10°/s、Pitch −10°/s），五组均存活 30 秒。当前可验证的鲁棒结果仍是2秒周期候选，尚未验证模型几何误差、舵机动态负载曲线和广泛地面变化，不能直接用于真机。

## 使用

仿真步态控制提供 `--roll-kp`、`--pitch-kp` 独立增益、正向 `--adaptive-kp` 和独立负向 `--adaptive-roll-negative` / `--adaptive-pitch-negative` 调度。配置正向 0.008、负向 Roll 0.001、负向 Pitch 0.003、roll bias 7° 后，四种 ±4°初态组合均存活 60 秒，前进 0.086–0.389 m，最大倾角不超过 19.48°。该结果证明了当前参数下仿真能抵抗四种指定初始偏置；增益调度仍是仿真探索项，不能视为真机控制参数。

足端 IK 模式在摆动腿上叠加世界坐标前移/抬脚目标。当前较优配置为 `--foot-ik --ik-blend 0.16 --step-length 0.007 --step-height 0.006`，采用 2.0 s 周期、roll bias 7°、髋 4°、膝 22°、踝 −9°、Kp/Kd 0.35/0.05、正向自适应增益 0.008、负向 Roll/Pitch 增益 0.001/0.003 和规格扭矩模型。四种 ±4° Roll/Pitch 初态组合都存活 60 秒，前进 0.314–0.448 m，最大倾角 21.17°。混合比例 0.15 的四组也存活 60 秒，但最差象限只前进 0.086 m；增至 0.17 后一个象限 16.6 秒倾倒，说明混合比例边界较窄。更大前移目标 8 mm 时四种象限只通过两种；7.5 mm 在两个困难象限分别 18.9 秒和 12.3 秒倾倒。0.16 / 7 mm / 6 mm 是目前四象限长测中综合前进和稳定性较好的配置。该方案仍沿用预设相位步态，仅对摆动腿做小幅足端修正，不等同于完整足端轨迹控制，也未经真机验证。

当前 0.16 配置再延长到 120 秒：`Roll/Pitch=−4°/+4°` 前进 0.802 m、最大倾角 20.19°；`−4°/−4°` 前进 0.374 m、最大倾角 21.17°，均未倾倒。其他两个 ±4°象限目前验证到 60 秒。

进一步提速验证：周期缩短至 1.9 s 时，两个 ±4°象限在约 10–14 秒倾倒；1.8 s 和 1.7 s 的失稳更快。固定 2.0 s 周期并把摆动占比改为 0.28，四象限均通过 30 秒，但速度无稳定提升；改为 0.32 时一个象限在 14.5 秒倾倒。因此目前仍采用 2.0 s / 0.30 摆动比。

幅度与伺服增益扫描：在 `Roll/Pitch=+4°/+4°` 下，髋幅增加到 4.5°、5°、5.5°、6°分别于约 9.6、9.2、14.8、5.2 秒倾倒。保持髋幅 4°，FW K=24 在 29.8 秒倾倒，K=28/32 在约 3.3 秒倾倒；K=16 存活 30 秒，但脚离地时间降到约 3%–8%。当前 K=20、髋幅 4°仍是稳定候选，单纯提高髋幅或位置环增益无法稳健提速。

扭矩敏感性复核：保持当前 7/6 mm IK 步态与自适应平衡参数，将 SimRobot 堵转扭矩/空载速度从 1.471 N·m / 4.55 rad/s 降到 1.35/4.3 和 1.2/4.0。两档下四种 ±4°组合初态都存活 30 秒；最差象限（Roll=Pitch=−4°）又分别通过 60 秒。1.35/4.3 时该象限前进 0.383 m、腾空约 7%–9%；1.2/4.0 时只前进 0.039 m、腾空约 5%–9%，表现为很慢的拖步。说明稳定性仍在，但有效步速显著受可用舵机扭矩/速度限制。

```bash
./return_init 3
./gait_step help
./gait_step next 1.5 5 5 10 -5
./gait_step reset 2
./gait_probe stand
./gait_probe shift_left 2 3
./gait_probe lift_right 2 3
./gait_probe home
BIN=./gait_probe MOVE_SEC=2 HOLD_SEC=2 SHIFT_DEG=3 LIFT_DEG=0 bash scripts/traditional_gait/probe_cycle.sh
./gait_imu_pd 0.25 0.025 5 2
```

## 下一步开发

1. 在有防摔保护的条件下验证站姿、左右重心转移、抬腿的方向和安全幅度，并修正关节目标映射。
2. 配置并验证 IMU 的轴映射、陀螺仪零偏和姿态角；把姿态 PD 与统一步态循环整合。
3. 已在仿真入口加入足端目标和雅可比数值 IK；继续调试支撑稳定后，再支持自动步态周期和可调支撑相时长。
4. 增加连续四阶段状态机、平滑启动/停止、通信超时和姿态异常保护。
5. 先在仿真验证，再进行吊挂真机验证；以上程序目前尚未经过编译和真机测试，不应视为已验证的行走控制器。

## 注意

- 从仓库根目录运行，程序才能找到 `jduck_tools/calibration.yaml`。
- 串口固定为 `/dev/ttyS3`。启动前确保没有其他程序同时占用舵机总线。
- 偏移值只是实验参数；检查方向时从小值开始并做好防摔保护。
- `.gait_phase` 是 `gait_step` 的运行状态文件，不应作为校准数据或动作配置使用。
