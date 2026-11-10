#!/usr/bin/env bash
# Long task (2026-11-10 night): train stock-firmware jduck walking policies until one passes the
# calibrated-sim gate, then stop. Two machines, each pulls the next untried config from a queue.
#
# Gate (jduck_tools/calib_stand_eval.py --walk, calibrated sim = robot replica):
#   zero cmd: leg target change < 0.5 °/tick, no fall
#   every direction at the board's key limits (vx 0.08, vy 0.05, wz 0.25): no fall, |yaw| < 30° (turns: within 30 % of ideal)
#   backward / forward 0.08 m/s: ≥ 0.6 m in 15 s; left / right 0.05 m/s: ≥ 0.4 m (2026-11-10 19:20: O4a stood
#   still at forward 0.08 and the yaw-only check let it through)
#
# Results: ~/jduck_ops/long_O/<name>.eval.txt, summary ~/jduck_ops/long_O/summary.txt,
# PASS → ~/jduck_ops/long_O/PASS_<name>.onnx and the loop ends on both machines.
set -u
OPS=~/jduck_ops; OUT=$OPS/long_O; mkdir -p $OUT; touch $OUT/.q
RL=~/rl_control/third_party/microduck_rl
R=navi@10.229.20.227; RRL=/home/navi/rl_control/third_party/microduck_rl
BASE="JDUCK_FW=orig JDUCK_OBS_ADC=1 JDUCK_OBS_LAG=1 JDUCK_HOME=old JDUCK_STAND_QUIET=4 JDUCK_STAND_QUIET_S2=1 JDUCK_ANG_EMA=0.5 JDUCK_KP_DR=0.85,1.15 JDUCK_TORQUE=1.2 JDUCK_NOLOAD=4.8 JDUCK_TORQUE_DR=0.95,1.3 JDUCK_FRICTION=0.005 JDUCK_FRICTION_DR=0,2"
CAPS="JDUCK_CAPS_S=5 JDUCK_CAPS_T=0.5 JDUCK_CAPS_WALK=0 JDUCK_CAPS_CALM=0.3"
# Queue: name|seed|extra env. Ordered by what the earlier runs suggest is most likely to help:
#   stronger standing quiet / CAPS (zero-cmd jitter), wider delay + gyro noise (robot robustness),
#   then different seeds of whatever worked best so far.
O5="JDUCK_CAPS_S=10 JDUCK_CAPS_T=1 JDUCK_CAPS_WALK=0 JDUCK_CAPS_CALM=0.3 JDUCK_ACT_DELAY=2,6 JDUCK_GYRO_NOISE=0.15 JDUCK_CMD=0.12,0.08,0.4 JDUCK_LIN_STD=0.06"
# O5*: the key-limit speeds (0.08 m/s) earned 85 % of the tracking reward standing still → the policy
# didn't step. Train on the board's speed range and a tight lin std (standing at 0.08 → 17 %).
O6="$O5 JDUCK_CMD_CURR=1500"
# O6*: O5a/O5b never stepped (air_time 0.01 throughout): the tight std removes the gradient before a
# gait exists. Learn to walk on the default ranges/stds first, switch to the board's at 1500.
O7="${O6/JDUCK_LIN_STD=0.06/JDUCK_LIN_STD=0.045} JDUCK_ANG_STD=0.12"
# O7*: O6a/O6b step at low speed now (fwd 0.5 m) but drift 60–110° in yaw and barely strafe:
# ang std 0.283 keeps 91 % of the yaw reward at a 75°/15 s drift, lin std 0.06 keeps 50 % standing
# still on a 0.05 m/s strafe. Tighten both after the 1500-iteration switch.
O8="$O6 JDUCK_ANG_STD=0.12 JDUCK_LIN_REL_W=2"
# O8*: O7a/O7b (lin std 0.045) kept the yaw straight (−11°) but stopped tracking speed:
# track_linear_velocity fell 1.2 → 0.26 after the switch, fwd 0.08 walked 0.24 m even in the
# training env. Keep lin std 0.06 + ang std 0.12, add a RELATIVE-error lin tracking term
# (standing on a 0.08 command → 0.02, 75 % speed → 0.78).
QUEUE=(
  "O8a|2|$O8"
  "O8b|1|$O8"
  "O8c|2|$O8 JDUCK_STAND_QUIET=8"
  "O8d|3|$O8"
  "O7a|2|$O7"
  "O7b|2|$O7 JDUCK_STAND_QUIET=8"
  "O7c|1|$O7"
  "O7d|3|$O7"
  "O6a|2|$O6"
  "O6b|1|$O6"
  "O6c|2|$O6 JDUCK_STAND_QUIET=8"
  "O6d|3|$O6"
  "O5a|2|$O5"
  "O5b|2|$O5 JDUCK_STAND_QUIET=8"
  "O5c|1|$O5"
  "O5d|3|$O5"
  "O4a|2|$CAPS JDUCK_ACT_DELAY=3,5 JDUCK_STAND_QUIET=8"
  "O4b|2|JDUCK_CAPS_S=10 JDUCK_CAPS_T=1 JDUCK_CAPS_WALK=0 JDUCK_CAPS_CALM=0.3 JDUCK_ACT_DELAY=2,6 JDUCK_GYRO_NOISE=0.15"
  "O4c|1|$CAPS JDUCK_ACT_DELAY=2,6 JDUCK_GYRO_NOISE=0.15"
  "O4d|2|$CAPS JDUCK_ACT_DELAY=2,6 JDUCK_GYRO_NOISE=0.15 JDUCK_STAND_QUIET=8"
  "O4e|3|$CAPS JDUCK_ACT_DELAY=2,6 JDUCK_GYRO_NOISE=0.15"
  "O4f|2|JDUCK_CAPS_S=10 JDUCK_CAPS_T=1 JDUCK_CAPS_WALK=0.3 JDUCK_CAPS_CALM=0.3 JDUCK_ACT_DELAY=2,6 JDUCK_GYRO_NOISE=0.15 JDUCK_STAND_QUIET=8"
  "O4g|1|JDUCK_CAPS_S=10 JDUCK_CAPS_T=1 JDUCK_CAPS_WALK=0 JDUCK_CAPS_CALM=0.3 JDUCK_ACT_DELAY=2,6 JDUCK_GYRO_NOISE=0.15 JDUCK_STAND_QUIET=8"
  "O4h|3|JDUCK_CAPS_S=10 JDUCK_CAPS_T=1 JDUCK_CAPS_WALK=0 JDUCK_CAPS_CALM=0.3 JDUCK_ACT_DELAY=2,6 JDUCK_GYRO_NOISE=0.15 JDUCK_STAND_QUIET=8"
)
log() { echo "[$(date +%m-%d_%T)] $*" | tee -a $OUT/summary.txt; }

# gate check on an eval file → prints PASS/FAIL and the reasons
gate() {
  python3 - "$1" <<'PY'
import re, sys
t = open(sys.argv[1]).read()
why = []
m = re.search(r"校准仿真 \S+\s+([\d.]+)", t)
if not m: print("FAIL 没有零速结果"); sys.exit()
jit = float(m.group(1))
if jit >= 0.5: why.append(f"零速抖 {jit:.2f}°≥0.5")
if re.search(r"校准仿真 .*摔", t): why.append("零速摔")
for name, lim in (("前进 0.08", None), ("后退 0.08", None), ("左移 0.05", None), ("右移 0.05", None), ("左转 0.25", 215), ("右转 0.25", -215)):
    line = next((l for l in t.splitlines() if l.strip().startswith(name)), "")
    if not line: why.append(f"{name} 无结果"); continue
    if "摔" in line: why.append(f"{name} 摔"); continue
    yaw = float(re.search(r"偏航\s+([+-]?\d+)", line).group(1))
    if lim is None and abs(yaw) >= 30: why.append(f"{name} 偏航 {yaw:+.0f}°")
    if lim is not None and abs(yaw - lim) > 0.3 * abs(lim): why.append(f"{name} 转了 {yaw:+.0f}°（理想 {lim:+d}°）")
    dx = float(re.search(r"dx\s+([+-][\d.]+)", line).group(1))
    dy = float(re.search(r"dy\s+([+-][\d.]+)", line).group(1))
    if name == "后退 0.08" and dx > -0.6: why.append(f"后退只走了 {dx:+.2f} m")
    if name == "前进 0.08" and dx < 0.6: why.append(f"前进只走了 {dx:+.2f} m")
    if name == "左移 0.05" and dy < 0.4: why.append(f"左移只走了 {dy:+.2f} m")
    if name == "右移 0.05" and dy > -0.4: why.append(f"右移只走了 {dy:+.2f} m")
print(("PASS" if not why else "FAIL ") + "；".join(why) + f"  [零速 {jit:.2f}°]")
PY
}

evaluate() {   # $1 name  $2 onnx
  ( cd $RL && timeout 3600 env -u PYTHONPATH uv run --frozen python ../../jduck_tools/calib_stand_eval.py "$2" --walk --seeds 2 ) \
      2>&1 | grep -vE "^\s*$|Warn" > $OUT/$1.eval.txt
  local g; g=$(gate $OUT/$1.eval.txt)
  log "$1: $g"
  if [[ "$g" == PASS* ]]; then cp "$2" $OUT/PASS_$1.onnx; touch $OUT/PASSED; fi
}

train_script() {   # $1 name $2 seed $3 extra  → writes ~/jduck_ops/$1.sh (same layout on both machines)
  cat > $OPS/$1.sh <<EOF
#!/usr/bin/env bash
cd ~/rl_control/third_party/microduck_rl
export PATH=\$HOME/.local/bin:\$PATH WANDB_MODE=offline
export $BASE $3
env -u PYTHONPATH uv run --frozen train Mjlab-Velocity-Flat-JDuck --env.scene.num-envs 4096 --agent.max_iterations 5000 \\
    --agent.seed $2 --agent.run-name $1 --agent.logger tensorboard > logs/pipeline/$1.train.log 2>&1
echo "== done \$(date +%F_%T)" > logs/pipeline/$1.done
EOF
}

export_run() {    # $1 name $2 where(desk|lap) → echoes onnx path
  local d
  if [ "$2" = desk ]; then
    local exp; exp=$(ssh $R "ls -d $RRL/logs/rsl_rl/jduck_velocity_*/*_$1 | head -1")
    local e; e=$(basename $(dirname "$exp"))
    mkdir -p $RL/logs/rsl_rl/$e
    rsync -a $R:$RRL/logs/rsl_rl/$e/ $RL/logs/rsl_rl/$e/ --include='*/' --include="*_$1/model_4999.pt" --include="*_$1/params/***" --exclude='*'
    d=$RL/logs/rsl_rl/$e/$(basename "$exp")
  else
    d=$(ls -d $RL/logs/rsl_rl/jduck_velocity_*/*_$1 | head -1)
  fi
  ( cd $RL && scripts/jduck_export.sh "$d" 4999 $OPS/$1_3999.onnx > $OPS/$1_export.log 2>&1 )
  echo $OPS/$1_3999.onnx
}

# ── 1. the two runs already going: O3a (desktop), O3b (laptop) ────────────────
wait_done() {  # $1 name $2 where
  if [ "$2" = desk ]; then until ssh $R "test -f $RRL/logs/pipeline/$1.done"; do sleep 60; done
  else until [ -f $RL/logs/pipeline/$1.done ]; do sleep 60; done; fi
}
run_slot() {   # $1 where; drains the queue for that machine
  local where=$1
  while [ ! -f $OUT/PASSED ]; do
    local item; item=$(flock $OUT/.q python3 - <<PY
import os
q = """$(printf '%s\n' "${QUEUE[@]}")""".strip().splitlines()
done = set(open("$OUT/.taken").read().split()) if os.path.exists("$OUT/.taken") else set()
nxt = next((x for x in q if x.split("|")[0] not in done), "")
if nxt:
    open("$OUT/.taken", "a").write(nxt.split("|")[0] + "\n")
print(nxt)
PY
)
    [ -z "$item" ] && { log "$where: 队列用完"; return; }
    IFS='|' read -r name seed extra <<<"$item"
    train_script $name $seed "$extra"
    log "$where 开始 $name（seed $seed：$extra）"
    if [ $where = desk ]; then
      rsync -a $RL/src/ $R:$RRL/src/ && rsync -a $RL/scripts/ $R:$RRL/scripts/ && scp -q $OPS/$name.sh $R:jduck_ops_$name.sh
      ssh $R "mkdir -p ~/jduck_ops; mv ~/jduck_ops_$name.sh ~/jduck_ops/$name.sh; nohup setsid bash ~/jduck_ops/$name.sh >/dev/null 2>&1 < /dev/null &"
    else
      nohup setsid bash $OPS/$name.sh >/dev/null 2>&1 < /dev/null &
    fi
    wait_done $name $where
    [ -f $OUT/PASSED ] && return
    evaluate $name "$(export_run $name $where)"
  done
}

first() {      # $1 name $2 where ("-" = nothing running: go straight to the queue)
  [ "$1" = "-" ] && return
  wait_done $1 $2
  [ -f $OUT/PASSED ] && return
  evaluate $1 "$(export_run $1 $2)"
}

DESK_FIRST=${1:-O3a}; LAP_FIRST=${2:-O3b}
log "长程任务开始：$DESK_FIRST（台式机）、$LAP_FIRST（笔记本）已在跑；队列 ${#QUEUE[@]} 个候选"
( first $DESK_FIRST desk; run_slot desk ) &
( first $LAP_FIRST lap;  run_slot lap ) &
wait
if [ -f $OUT/PASSED ]; then log "完成：$(ls $OUT/PASS_*.onnx)"; else log "队列全部练完，没有达标的"; fi
