#!/usr/bin/env bash
# Export and evaluate N6home3k4a after its 4000-iteration run finishes.
set -euo pipefail
name=${1:-N6home3k4a}
location=${2:-desk}
rl="$HOME/rl_control/third_party/microduck_rl"
out="$HOME/jduck_ops/n6"
mkdir -p "$out"
if [ "$location" = desk ]; then
  remote=navi@10.229.20.227
  remote_rl=/home/navi/rl_control/third_party/microduck_rl
  until ssh -o BatchMode=yes -o ConnectTimeout=10 "$remote" "test -f '$remote_rl/logs/pipeline/$name.done'" 2>/dev/null; do sleep 60; done
  remote_run=$(ssh "$remote" "find '$remote_rl/logs/rsl_rl' -maxdepth 3 -type f -path '*_$name/model_3999.pt' -printf '%h\\n' | head -1")
  test -n "$remote_run"
  run="$rl/logs/rsl_rl/$(basename "$(dirname "$remote_run")")/$(basename "$remote_run")"
  mkdir -p "$run"
  rsync -a --include='*/' --include='model_3999.pt' --include='params/***' --exclude='*' \
    "$remote:$remote_run/" "$run/"
else
  until [ -f "$rl/logs/pipeline/$name.done" ]; do sleep 60; done
  run=$(find "$rl/logs/rsl_rl" -maxdepth 3 -type f -path "*_$name/model_3999.pt" -printf '%h\n' | head -1)
fi
test -n "$run"
cd "$rl"
scripts/jduck_export.sh "$run" 3999 "$out/$name.onnx" > "$out/$name.export.log" 2>&1
for stall in 1.25 1.4 1.55; do
  env -u PYTHONPATH timeout 3600 uv run --frozen python ../../jduck_tools/calib_stand_eval.py "$out/$name.onnx" \
    --fw 4,0,0 --walk --seed0 5 --seeds 4 --stall "$stall" > "$out/$name.t$stall.txt" 2>&1
done
for stall in 1.25 1.4 1.55; do
  env -u PYTHONPATH timeout 3600 uv run --frozen python ../../jduck_tools/calib_stand_eval.py "$out/$name.onnx" \
    --fw 4,0,0 --walk --stand-hold --seed0 5 --seeds 4 --stall "$stall" > "$out/$name.hold.t$stall.txt" 2>&1
done
echo "[$(date +%F_%T)] $name 测评完" >> "$out/log.txt"
