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
BASE="JDUCK_FW=orig JDUCK_OBS_ADC=1 JDUCK_OBS_LAG=1 JDUCK_HOME=old JDUCK_STAND_QUIET=4 JDUCK_STAND_QUIET_S2=1 JDUCK_ANG_EMA=0.5 JDUCK_KP_DR=0.85,1.15 JDUCK_TORQUE=1.2 JDUCK_NOLOAD=4.8 JDUCK_FRICTION=0.005 JDUCK_FRICTION_DR=0,2"
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
O9="${O8/JDUCK_CAPS_S=10/JDUCK_CAPS_S=20} JDUCK_CAPS_STAND=8"
# O9*: O8a/O8b walk at the key speeds now (fwd ~1.0 m, back ~1.1 m, left 0.8–0.9 m, yaw mostly
# 7–37°); the main blocker is zero-command jitter 0.89–0.93°/tick (gate 0.5). Double the CAPS
# spatial weight and the zero-command CAPS boost (smoothness only where the jitter shows).
O10="$O8 JDUCK_TORQUE_DR=0.67,1.33"
# O10*: O8d (passed at 1.4 N·m) stood leaning back and fell at ≤ 0.96 N·m — its training torque was
# 1.14–1.56. Train on 0.8–1.6 N·m (1.2 × 0.67–1.33) and gate at 0.9 AND 1.4.
O11="$O8 JDUCK_TORQUE=0.96"
# O11*: O8 recipe on the spec-sheet servo (0.96 N·m, default DR 0.8–1.15× like O1). BASE has
# JDUCK_TORQUE=1.2; the later JDUCK_TORQUE=0.96 wins. Gate at 0.8 and 0.96 + posture ≤ 5°.
O12="$O11 JDUCK_TORQUE_DR=0.9,1.3 JDUCK_CMD_STAGED=1"
# O12*: robot log 2026-11-11 17:20 — O1 at 8.1 V jitters 40 % more than at ~7.2 V (same 4 Hz, higher gain);
# torque 0.96 × 0.9–1.3 ≈ 0.86–1.25 N·m spans a full-to-flat battery. Staged switch (O11a fell 0.38 → 2.5
# at the one-shot switch). Gate at 0.86 and 1.25 + stance ≤ 5°.
O13="$O12 JDUCK_POSE_STAND_STD=0.07"
# O13*: O12a walks straight in every direction at both torques, zero-cmd 0.36° (0.86) / 0.60° (1.25), but
# stands with hip pitch ±8–10° and bent knees. The standing pose std 0.15 keeps 42 % of the pose term at
# 8° off; 0.07 keeps 2 %.
O14="$O13 JDUCK_STANCE_W=2"
# O14*: three stance failures in a row (O12a/O12b/O13a: one hip pitch 7–15° off). The stock pose term averages
# over 10 leg joints → one joint 12° off keeps 41 % at std 0.07, weight 1. Add a per-joint PRODUCT stance term
# (hip pitch + knee, std 0.07, zero command only): that joint alone → 0.
O15="${O14/JDUCK_CAPS_S=10/JDUCK_CAPS_S=20} JDUCK_CAPS_STAND=8"
# O15*: zero-speed jitter failed three times in a row (O14a/b/c: 0.54–0.60° at 0.86 N·m, 0.81–0.88° at 1.25),
# worse as the stance got tighter (O12 0.36 → O14 0.57). Not the quiet term's S2: the reward sees the sampled
# action, and with action std 0.25 Σ Δa² ≈ 1.75 is all exploration noise — S2 0.003 would zero it for good.
# CAPS acts on the policy MEAN: O9a (S20, stand boost 8) was the calmest so far (0.18°). Raise S, boost and T.
O16="$O15 JDUCK_HEADING_W=2"
# O16*: yaw failed five times in a row (O14a–e: 30–74° in the 15 s key-speed walks). A steady 0.066 rad/s
# error keeps exp(−0.066²/0.12²) = 74 % of the yaw-rate term. heading_hold integrates the yaw error since
# the last command change: std 0.2 rad → 57° gives 0, 17° gives 10 %.
O17="$O16 JDUCK_STANCE_ALL=1 JDUCK_STANCE_STD=0.1 JDUCK_SYMMETRY=0.5"
# O17*: O15b passed the 5° hip-pitch/knee stance check but stood asymmetric (left hip roll −4~−10°, ankle +7°,
# right hip pitch +5°) and fell on the robot with the right foot forward. Stance product over all 10 leg joints
# (std 0.1: one joint 3° off keeps 76 %, all ten 3° off keep 6 %) + left/right mirror loss. Gate: ≤ 3° on all.
O18="$O17 JDUCK_STANCE_L1=4"
# O18*: O17b stood 3.5° mean off HOME per leg joint (left hip roll −8~−9°, left hip pitch −7~−8°) where the
# 10-joint product is exp(−Σe²/0.1²) = 0.005: flat, no pull back (stance_product 0.04 all run). Add an L1
# penalty −w·Σ|q−HOME| at zero command: same slope at any distance; 3.5° mean → −4·0.61 = −2.4 per step.
O19="$O18 JDUCK_STAND_TILT=10"
# O19*: O18a stood leaning back 2.9° in the calibrated sim (O1 0.2°) and 7.4° on the robot → fell backwards
# easily. `upright` (std² 0.05) pays 95 % at 3°, 74 % at 7°. Add −w·(|pitch|+|roll|) at zero command:
# 3° → −0.52 per step at w 10. Gate: |trunk pitch| ≤ 1.5° at zero command.
O20="$O19 JDUCK_TORQUE=1.4 JDUCK_TORQUE_DR=0.9,1.15"
# O20*: 2026-11-12 --hold-home on the robot: every leg joint sags 2 ADC counts (0.7–0.9°), the calibrated
# sim at 1.4 N·m sags the same, at 0.96 it COLLAPSES. O11–O19 trained on 0.96 × [0.9, 1.3] and were gated
# at 0.86 — a servo much softer than the robot's. Back to the fit: 1.4 × [0.9, 1.15], gate at 1.25 / 1.55.
# The IMU's 1.8° mounting pitch is corrected on the board (imu_spi.yaml pitch_offset_deg), not in training.
O21="${O12/JDUCK_TORQUE=0.96/JDUCK_TORQUE=1.4} JDUCK_NOLOAD=5.5 JDUCK_ACT_DELAY=1,3 JDUCK_HEADING_W=2 JDUCK_SYMMETRY=0.5"
O21="${O21/JDUCK_TORQUE_DR=0.9,1.3/JDUCK_TORQUE_DR=0.9,1.15}"
# O21*: zero speed is no longer the policy's job (jduck_run --stand-hold holds HOME; O20a–c jittered
# 0.9–1.5° at 1.4 N·m whatever the stand rewards). Walk only: O12 recipe (no stance/tilt/L1 terms, the
# best walker of the stock-firmware runs) + servo fit from the 2026-11-12 step-test (1.4 N·m, no-load
# 5.5 rad/s, +5 ms = 1 physics step, so ACT_DELAY 1–3) + heading hold + mirror loss for straight walks.
O22="$O21 JDUCK_YAW_REL_W=2"
# O22*: turning overshoot three times in a row (O21a/b/c: +25–60 % at the 0.25 rad/s key). The absolute yaw term
# (std 0.12) still pays 68 % at +30 %, 50 % at +40 %. Relative yaw tracking (rel_std 0.25): +30 % → 24 %, +40 % → 8 %.
# O23*: O21b (O21, seed 2) + --wz-scale 0.8 passes at 1.4/1.55 N·m; the O22 yaw-rel term did not beat it
# (O22a–d: forward 0.08 stalls or drifts 48–79°). More O21 seeds, gated the way the robot runs it (wz × 0.8).
QUEUE=(
  "O23a|5|$O21"
  "O23b|6|$O21"
  "O23c|7|$O21"
  "O23d|8|$O21"
  "O22a|2|$O22"
  "O22b|1|$O22"
  "O22c|3|$O22 JDUCK_ANG_STD=0.08"
  "O22d|4|${O22/JDUCK_YAW_REL_W=2/JDUCK_YAW_REL_W=4}"
  "O21a|1|$O21"
  "O21b|2|$O21"
  "O21c|3|$O21 JDUCK_ANG_STD=0.08"
  "O21d|4|$O21"
  "O20a|1|$O20"
  "O20b|2|$O20"
  "O20c|3|${O20/JDUCK_STAND_TILT=10/JDUCK_STAND_TILT=20}"
  "O20d|2|${O20/JDUCK_STANCE_L1=4/JDUCK_STANCE_L1=8}"
  "O19a|1|$O19"
  "O19b|2|$O19"
  "O19c|3|${O19/JDUCK_STAND_TILT=10/JDUCK_STAND_TILT=20}"
  "O19d|1|${O19/JDUCK_STANCE_L1=4/JDUCK_STANCE_L1=8}"
  "O17a|2|$O17"
  "O17b|1|$O17"
  "O18a|1|$O18"
  "O18b|2|$O18"
  "O18c|3|${O18/JDUCK_STANCE_L1=4/JDUCK_STANCE_L1=8}"
  "O18d|2|${O18/JDUCK_STANCE_L1=4/JDUCK_STANCE_L1=8}"
  "O17c|3|${O17/JDUCK_SYMMETRY=0.5/JDUCK_SYMMETRY=1}"
  "O17d|2|${O17/JDUCK_STANCE_STD=0.1/JDUCK_STANCE_STD=0.07}"
  "O17e|3|${O17/JDUCK_STANCE_W=2/JDUCK_STANCE_W=4}"
  "O15a|3|$O15"
  "O15b|1|${O15/JDUCK_CAPS_T=1/JDUCK_CAPS_T=3}"
  "O16a|2|$O16"
  "O16b|1|$O16 JDUCK_HEADING_STD=0.3"
  "O16c|3|$O16"
  "O15c|2|${O15/JDUCK_CAPS_T=1/JDUCK_CAPS_T=3}"
  "O15d|3|${O15/JDUCK_CAPS_STAND=8/JDUCK_CAPS_STAND=16}"
  "O14a|3|$O14"
  "O14b|2|$O14"
  "O14c|1|$O14"
  "O14d|3|${O14/JDUCK_STANCE_W=2/JDUCK_STANCE_W=4}"
  "O14e|2|${O14/JDUCK_CAPS_S=10/JDUCK_CAPS_S=20} JDUCK_CAPS_STAND=8"
  "O13a|3|$O13"
  "O13b|2|$O13"
  "O13c|1|$O13"
  "O13d|4|$O13"
  "O13e|3|${O13/JDUCK_CAPS_S=10/JDUCK_CAPS_S=20} JDUCK_CAPS_STAND=8"
  "O13f|2|${O13/JDUCK_CAPS_S=10/JDUCK_CAPS_S=20} JDUCK_CAPS_STAND=8"
  "O13g|3|${O13/JDUCK_POSE_STAND_STD=0.07/JDUCK_POSE_STAND_STD=0.05}"
  "O13h|2|${O13/JDUCK_POSE_STAND_STD=0.07/JDUCK_POSE_STAND_STD=0.05}"
)


log() { echo "[$(date +%m-%d_%T)] $*" | tee -a $OUT/summary.txt; }

# gate check on an eval file → prints PASS/FAIL and the reasons
gate() {
  python3 - "$1" <<'PY'
import re, sys
t = open(sys.argv[1]).read()
why = []
pm = re.search(r"站姿 .*最大偏离 ([\d.]+)°", t)
if pm and float(pm.group(1)) > 3.0:
    why.append(f"站姿偏离 HOME {float(pm.group(1)):.1f}°>3")   # O8d bent knees ±7–10°, tipped forward
pt = re.search(r"躯干俯仰 ([+-][\d.]+)°", t)
if pt and abs(float(pt.group(1))) > 1.5:
    why.append(f"零速躯干俯仰 {float(pt.group(1)):+.1f}°（|·|>1.5）")   # O18a −2.9° → fell backwards on the robot
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
for l in t.splitlines():                        # --stand-hold: walk 4 s, stop, must stay up
    if l.strip().startswith("停 ") and "摔" in l: why.append(l.strip())
print(("PASS" if not why else "FAIL ") + "；".join(why) + f"  [零速 {jit:.2f}°]")
PY
}

evaluate() {   # $1 name  $2 onnx — must pass at BOTH servo torques (2026-11-11: O8d passed at 1.4 N·m,
               # leaned back / fell at ≤ 0.96 on the robot)
  local all=PASS msg=""   # 2026-11-11 evening: torque back to the spec sheet (0.96 N·m) — O8d/O10 trained on 1.2 learned a bent-knee stance that tips on the robot
  for st in 1.25 1.55; do   # 2026-11-12 hold-home: real leg sag = sim at 1.4 N·m (0.96 collapses)
    ( cd $RL && timeout 3600 env -u PYTHONPATH uv run --frozen python ../../jduck_tools/calib_stand_eval.py "$2" --walk --stand-hold --wz-scale 0.8 --seeds 2 --stall $st ) \
        2>&1 | grep -vE "^\s*$|Warn" > $OUT/$1.eval_t$st.txt
    local g; g=$(gate $OUT/$1.eval_t$st.txt)
    msg+="[力矩 $st] $g  "
    [[ "$g" == PASS* ]] || all=FAIL
  done
  log "$1: $all  $msg"
  if [ $all = PASS ]; then cp "$2" $OUT/PASS_$1.onnx; touch $OUT/PASSED; fi
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
