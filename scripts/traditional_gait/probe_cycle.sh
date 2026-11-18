#!/usr/bin/env bash
# Conservative staged probe; lift offset defaults to 0 until calibrated.
set -euo pipefail
BIN="${BIN:-./gait_probe}"
MOVE_SEC="${MOVE_SEC:-2}"
HOLD_SEC="${HOLD_SEC:-2}"
SHIFT_DEG="${SHIFT_DEG:-3}"
LIFT_DEG="${LIFT_DEG:-0}"

run() { "$BIN" "$1" "$MOVE_SEC" "$2"; }
hold() { sleep "$HOLD_SEC"; }

run stand 0
hold
run shift_left "$SHIFT_DEG"
hold
run lift_right "$LIFT_DEG"
hold
run home 0
hold
run shift_right "$SHIFT_DEG"
hold
run lift_left "$LIFT_DEG"
hold
run home 0
