#!/usr/bin/env bash
# 泰山派（Debian 12 / Python 3.11 / aarch64）上一次性准备运行环境。离线也能装。
#
#   jduck_tools/setup_board.sh [WHEEL_DIR]
#
# 1. 用系统 python3 建 venv（默认 /userdata/jduck-venv，可用 $JDUCK_VENV 改），从 WHEEL_DIR 离线装包：
#    numpy 1.26.4、onnxruntime 1.20.1、pyserial 3.5、pyyaml 6.0.3、smbus2 及依赖。
#    WHEEL_DIR 默认 /userdata/jduck-runtime/a8df940/wheels（codex 2026-11-09 下好的 14 个 wheel，35 MB）。
#    没有 wheel 又能上网时：pip download 那 5 个包（--platform manylinux2014_aarch64 --python-version 311）。
# 2. 编译 ICM-42688 的 SPI 读数库 jduck_tools/libjduck_icm.so（gcc；源码 jduck_tools/icm42688/）。
# 3. 自检：import 各个包、打开 IMU 读一次重力。
set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
VENV=${JDUCK_VENV:-/userdata/jduck-venv}
WHEELS=${1:-/userdata/jduck-runtime/a8df940/wheels}

if [ ! -x "$VENV/bin/python" ]; then
  echo "== 建 venv：$VENV"
  python3 -m venv --without-pip "$VENV"
fi
if ! "$VENV/bin/python" -c "import numpy, onnxruntime, serial, yaml" 2>/dev/null; then
  [ -d "$WHEELS" ] || { echo "没有 wheel 目录 $WHEELS" >&2; exit 1; }
  echo "== 从 $WHEELS 离线装包"
  PIPWHL=$(ls "$WHEELS"/pip-*.whl | head -1)
  "$VENV/bin/python" "$PIPWHL/pip" install --no-index --find-links "$WHEELS" \
      pip numpy==1.26.4 onnxruntime==1.20.1 pyserial pyyaml smbus2
fi

echo "== 编译 libjduck_icm.so"
gcc -O2 -shared -fPIC -I"$REPO/jduck_tools/icm42688" -o "$REPO/jduck_tools/libjduck_icm.so" \
    "$REPO/jduck_tools/icm42688/spi_adapter.c"

echo "== 自检"
cd "$REPO/jduck_tools"
"$VENV/bin/python" - <<'PY'
import math, numpy, onnxruntime, serial, yaml
print("numpy", numpy.__version__, "| onnxruntime", onnxruntime.__version__, "| pyserial", serial.__version__)
from imu_spi_board import Imu
imu = Imu.from_config("imu_spi.yaml")
for _ in range(20):
    w, g = imu.read()
print("IMU 重力", [round(float(x), 3) for x in g], "倾斜 %.1f°" % math.degrees(math.acos(max(-1, min(1, -float(g[2]))))))
PY
echo "好了。启动：jduck_tools/run_robot.sh"
