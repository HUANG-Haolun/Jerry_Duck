#!/usr/bin/env bash
# $1 名字  $2 种子  $3 K   —— 修改版固件 + 2026-11-12 阶跃测试拟合的真舵机参数
cd ~/rl_control/third_party/microduck_rl
export PATH=$HOME/.local/bin:$PATH WANDB_MODE=offline
export JDUCK_FW=$3,0,0 JDUCK_TORQUE=1.4 JDUCK_TORQUE_DR=0.9,1.15 JDUCK_NOLOAD=5.5 JDUCK_ACT_DELAY=1,3 \
  JDUCK_OBS_ADC=1 JDUCK_OBS_LAG=1 JDUCK_HOME=old JDUCK_STAND_QUIET=4 JDUCK_STAND_QUIET_S2=1 JDUCK_ANG_EMA=0.5 \
  JDUCK_KP_DR=0.85,1.15 JDUCK_FRICTION=0.005 JDUCK_FRICTION_DR=0,2 \
  JDUCK_CAPS_S=5 JDUCK_CAPS_T=0.5 JDUCK_CAPS_WALK=0 JDUCK_CAPS_CALM=0.3 JDUCK_GYRO_NOISE=0.15 \
  JDUCK_CMD=0.12,0.08,0.4 JDUCK_LIN_STD=0.06 JDUCK_CMD_CURR=1500 JDUCK_CMD_STAGED=1 JDUCK_ANG_STD=0.12 JDUCK_LIN_REL_W=2 \
  JDUCK_HEADING_W=2 JDUCK_SYMMETRY=0.5
mkdir -p logs/pipeline
env -u PYTHONPATH uv run --frozen train Mjlab-Velocity-Flat-JDuck --env.scene.num-envs 4096 --agent.max_iterations 5000 \
    --agent.seed $2 --agent.run-name $1 --agent.logger tensorboard > logs/pipeline/$1.train.log 2>&1
echo "== done $(date +%F_%T)" > logs/pipeline/$1.done
