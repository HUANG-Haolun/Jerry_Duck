#!/usr/bin/env bash
# N3k4a: isolate stronger CAPS smoothing against N2k4a; keep all other settings.
set -euo pipefail
name=${1:-N3k4a}
seed=${2:-1}
iterations=${3:-4000}
num_envs=${4:-4096}
cd "$HOME/rl_control/third_party/microduck_rl"
export PATH="$HOME/.local/bin:$PATH" WANDB_MODE=offline
export JDUCK_FW=4,0,0 JDUCK_TORQUE=1.4 JDUCK_TORQUE_DR=0.9,1.15 JDUCK_NOLOAD=5.5 JDUCK_ACT_DELAY=1,3 \
  JDUCK_OBS_ADC=1 JDUCK_OBS_LAG=1 JDUCK_HOME=old JDUCK_STAND_QUIET=4 JDUCK_STAND_QUIET_S2=1 JDUCK_ANG_EMA=0.5 \
  JDUCK_KP_DR=0.85,1.15 JDUCK_FRICTION=0.005 JDUCK_FRICTION_DR=0,2 \
  JDUCK_CAPS_S=10 JDUCK_CAPS_T=1 JDUCK_CAPS_WALK=0 JDUCK_CAPS_CALM=0.3 JDUCK_GYRO_NOISE=0.15 \
  JDUCK_CMD=0.12,0.08,0.4 JDUCK_LIN_STD=0.06 JDUCK_CMD_CURR=1500 JDUCK_CMD_STAGED=1 JDUCK_ANG_STD=0.12 JDUCK_LIN_REL_W=2 \
  JDUCK_HEADING_W=2 JDUCK_SYMMETRY=0.5
mkdir -p logs/pipeline
env -u PYTHONPATH uv run --frozen train Mjlab-Velocity-Flat-JDuck --env.scene.num-envs "$num_envs" --agent.max_iterations "$iterations" \
  --agent.seed "$seed" --agent.run-name "$name" --agent.logger tensorboard > "logs/pipeline/$name.train.log" 2>&1
echo "== done $(date +%F_%T)" > "logs/pipeline/$name.done"
