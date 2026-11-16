#!/usr/bin/env bash
# $1 名字 $2 K $3 desk|lap —— 等训练完 → 导出 → 校准仿真测评（1.25/1.4/1.55 N·m，种子 5–8）
set -u
N=$1; K=$2; W=$3; R=navi@10.229.20.227; RRL=/home/navi/rl_control/third_party/microduck_rl
RL=~/rl_control/third_party/microduck_rl; OUT=~/jduck_ops/n2
if [ $W = desk ]; then
  until ssh -o ConnectTimeout=20 $R "test -f $RRL/logs/pipeline/$N.done" 2>/dev/null; do sleep 60; done
  exp=$(ssh $R "ls -d $RRL/logs/rsl_rl/jduck_velocity_*/*_$N | head -1"); e=$(basename $(dirname "$exp"))
  mkdir -p $RL/logs/rsl_rl/$e
  rsync -a $R:$RRL/logs/rsl_rl/$e/ $RL/logs/rsl_rl/$e/ --include='*/' --include="*_$N/model_4999.pt" --include="*_$N/params/***" --exclude='*'
  d=$RL/logs/rsl_rl/$e/$(basename "$exp")
else
  until [ -f $RL/logs/pipeline/$N.done ]; do sleep 60; done
  d=$(ls -d $RL/logs/rsl_rl/jduck_velocity_*/*_$N | head -1)
fi
cd $RL && scripts/jduck_export.sh "$d" 4999 $OUT/$N.onnx > $OUT/$N.export.log 2>&1
for st in 1.25 1.4 1.55; do
  ( env -u PYTHONPATH timeout 3600 uv run --frozen python ../../jduck_tools/calib_stand_eval.py $OUT/$N.onnx --fw $K,0,0 --walk \
      --seed0 5 --seeds 4 --stall $st 2>&1 | grep -vE "^\s*$|Warn|warn|Patch" > $OUT/$N.t$st.txt ) &
done
wait
echo "[$(date +%m-%d_%H:%M:%S)] $N 测评完" >> $OUT/log.txt
