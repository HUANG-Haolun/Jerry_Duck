# jduck 起身策略的制作脚本

都在 `third_party/microduck_rl` 下用 `uv run --frozen python ../../scripts/getup/<脚本>` 运行，
运行前要 export 训练用的 `JDUCK_*` 变量（见下面 PPO 那条命令）。脚本里的中间文件读写 `~/jduck_ops/`。

| 脚本 | 做什么 | 输出 |
|---|---|---|
| `cem_env.py down|roll [噪声] [代数]` | 训练环境里 CEM 搜关键帧动作（4 帧，左右腿可不同），按 32 个随机化+噪声环境的站起率打分 | `~/jduck_ops/cem_env_<down|roll>.json` |
| `record_env.py out.npz down.json [roll.json] [噪声]` | 带噪声回放、录下成功回合的 (qpos, qvel, 动作) | `env_states_<down|up>.npz` |
| `dagger_getup.py base.pt out.pt` | DAgger：专家 = 最近录制状态的下一步动作；β 从 1 降到 0 | 可接着 PPO 的 checkpoint |
| `getup_eval.py <task> <onnx> [秒] [状态列表]` | 各种起始姿态（站、蹲、趴、仰、侧躺）的站起率 | 打印 |
| `getup_video.py <task> <onnx> <mp4>` | 录起身视频（`MUJOCO_GL=egl`） | mp4 |

然后 PPO 微调（台式机上 1250 次迭代约 20 分钟）：
```bash
MICRODUCK_WARM_START=1 JDUCK_IMIT_W=2 JDUCK_REPLAY_PROB=0.3 JDUCK_REPLAY_FILE=~/jduck_ops/env_states_both.npz \
JDUCK_HEAD_COST=0.1 JDUCK_FW=8,0,0 JDUCK_TORQUE=0.96 JDUCK_NOLOAD=4.8 JDUCK_HOME=old JDUCK_OBS_ADC=1 JDUCK_OBS_LAG=1 \
JDUCK_KP_DR=0.9,1.25 JDUCK_TORQUE_DR=0.95,1.15 \
uv run --frozen train Mjlab-StandUp-Flat-JDuck --env.scene.num-envs 4096 --agent.max_iterations 1250 \
    --agent.resume True --agent.load-run 0000_dagger --agent.load-checkpoint model_0.pt
```
（`0000_dagger/model_0.pt` = `dagger_getup.py` 的输出，要放进这个实验名的日志目录里。）
