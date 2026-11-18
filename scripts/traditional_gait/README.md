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

## 编译

在仓库根目录执行：

```bash
gcc -O2 -Wall scripts/traditional_gait/return_init.c -lm -o return_init
gcc -O2 -Wall scripts/traditional_gait/gait_step.c -lm -o gait_step
gcc -O2 -Wall scripts/traditional_gait/gait_probe.c -lm -o gait_probe
gcc -O2 -Wall scripts/traditional_gait/gait_imu_pd.c jduck_tools/icm42688/spi_adapter.c -lm -o gait_imu_pd
```

## 使用

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
3. 根据 URDF 关节几何实现足端轨迹和逆运动学，支持可调步长、步高、周期及支撑相时长。
4. 增加连续四阶段状态机、平滑启动/停止、通信超时和姿态异常保护。
5. 先在仿真验证，再进行吊挂真机验证；以上程序目前尚未经过编译和真机测试，不应视为已验证的行走控制器。

## 注意

- 从仓库根目录运行，程序才能找到 `jduck_tools/calibration.yaml`。
- 串口固定为 `/dev/ttyS3`。启动前确保没有其他程序同时占用舵机总线。
- 偏移值只是实验参数；检查方向时从小值开始并做好防摔保护。
- `.gait_phase` 是 `gait_step` 的运行状态文件，不应作为校准数据或动作配置使用。
