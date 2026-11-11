#!/usr/bin/env bash
# jduck 真机启动（泰山派上运行）。等价于 2026-11-09 晚上手敲的那条长命令，参数都有默认值。
#
#   jduck_tools/run_robot.sh                          # 默认：O1 模型（原版固件），键盘控制
#   jduck_tools/run_robot.sh -m models/xxx.onnx       # 换模型
#   jduck_tools/run_robot.sh --hold-home              # 只移到 HOME 并保持（不跑策略）
#   jduck_tools/run_robot.sh --log /userdata/a.npz    # 录每一拍的数据（给 jitter_compare.py / real_calib.py）
#   jduck_tools/run_robot.sh -- --max-ticks 1250      # `--` 之后的参数原样传给 jduck_run.py
#
# 选项：
#   -m, --model PATH     策略 ONNX（默认 models/jduck_O1_orig_full.onnx）
#   --vx/--vy/--wz V     按键速度上限（默认 0.08 / 0.05 / 0.25）
#   --tilt DEG           倾斜超过就卸力（默认 20）
#   --ramp S             上电后移到 HOME 用几秒（默认 10）
#   --blend S            策略输出渐入秒数（默认 1；5 秒时缩小的策略动作在力矩 0.96 N·m 下连 O1 都会摔）
#   --hold-home          只保持 HOME，不执行策略
#   --log PATH           录数据到 .npz
#   -y, --yes            跳过“继续？[y/N]”确认
#
# Python 用 jduck_tools/setup_board.sh 装好的 venv（默认 $JDUCK_VENV = /userdata/jduck-venv，
# 没有就退回 /userdata/jduck-runtime/a8df940/venv —— 2026-11-09 codex 在板子上建的那个）。
set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
cd "$REPO"

MODEL=models/jduck_O1_orig_full.onnx
VX=0.08; VY=0.05; WZ=0.25; TILT=20; RAMP=10; BLEND=1
EXTRA=()
while [ $# -gt 0 ]; do
  case "$1" in
    -m|--model) MODEL=$2; shift 2 ;;
    --vx) VX=$2; shift 2 ;;
    --vy) VY=$2; shift 2 ;;
    --wz) WZ=$2; shift 2 ;;
    --tilt) TILT=$2; shift 2 ;;
    --ramp) RAMP=$2; shift 2 ;;
    --blend) BLEND=$2; shift 2 ;;
    --hold-home) EXTRA+=(--hold-home); shift ;;
    --log) EXTRA+=(--log "$2"); shift 2 ;;
    -y|--yes) EXTRA+=(--yes); shift ;;
    -h|--help) sed -n 2,20p "$0"; exit 0 ;;
    --) shift; EXTRA+=("$@"); break ;;
    *) echo "不认识的参数：$1（-h 看用法）" >&2; exit 1 ;;
  esac
done

PY=""
for v in "${JDUCK_VENV:-/userdata/jduck-venv}" /userdata/jduck-runtime/a8df940/venv; do
  if [ -x "$v/bin/python" ]; then PY="$v/bin/python"; break; fi
done
[ -n "$PY" ] || { echo "找不到 Python 环境：先运行 jduck_tools/setup_board.sh" >&2; exit 1; }
[ -f jduck_tools/libjduck_icm.so ] || { echo "缺 jduck_tools/libjduck_icm.so：先运行 jduck_tools/setup_board.sh" >&2; exit 1; }
[ -f "$MODEL" ] || { echo "找不到模型 $MODEL" >&2; exit 1; }

echo "模型 $MODEL | 速度上限 vx $VX vy $VY wz $WZ | 倾斜保护 ${TILT}° | Python $PY"
OPENBLAS_NUM_THREADS=1 exec "$PY" -u jduck_tools/jduck_run.py \
  --port /dev/ttyS3 --baud 230460 \
  --imu-config jduck_tools/imu_spi.yaml \
  --policy "$MODEL" \
  --ramp-s "$RAMP" --policy-blend-s "$BLEND" \
  --max-vx "$VX" --max-vy "$VY" --max-wz "$WZ" \
  --head-rate-div 2 --max-tilt "$TILT" \
  "${EXTRA[@]}"
