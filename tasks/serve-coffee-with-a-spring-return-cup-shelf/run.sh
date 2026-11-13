#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "$0")" && pwd)"
mode="${1:-check}"
cd "$root"
if [[ "$mode" == check ]]; then python3 validate_local.py; exit; fi
source "${YAM_ENV:-/data/IsaacLab-Arena/yam.env}"
export PYTHONPATH="$root:${PYTHONPATH:-}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 DEMO_TIME_SCALE=1
unset SETTLE_ONLY ABLATION GRASP_TEST GRASP_FILLED DIAGNOSTIC_NO_CAMERAS DIAGNOSTIC_FULL_SEQUENCE
case "$mode" in
  camera_check) export SETTLE_ONLY=1; policy=coffee_servo; steps=30; width=1280; height=720 ;;
  sequence_test) export DIAGNOSTIC_NO_CAMERAS=1 DIAGNOSTIC_FULL_SEQUENCE=1; policy=coffee_servo; steps=2400; width=320; height=180 ;;
  grip_filled) export GRASP_TEST=1 GRASP_FILLED=1 DIAGNOSTIC_NO_CAMERAS=1; policy=coffee_servo; steps=500; width=320; height=180 ;;
  grip_test) export GRASP_TEST=1 DIAGNOSTIC_NO_CAMERAS=1; policy=coffee_servo; steps=500; width=320; height=180 ;;
  probe) export SETTLE_ONLY=1; policy=coffee_servo; steps=200; width=320; height=180 ;;
  pilot) policy=coffee_servo; steps=400; width=1280; height=720 ;;
  demo) policy=coffee_servo; steps=2400; width=1280; height=720 ;;
  dream)
    python3 evidence.py qualify "${QUALIFIED_DEMO:?Set QUALIFIED_DEMO to a successful current-source demo trial JSON}"
    policy="${DREAM_POLICY:-dream}"; steps=2400; width=1280; height=720 ;;
  *) echo 'Use check, probe, pilot, demo or dream' >&2; exit 2 ;;
esac
base="${RUNS_ROOT:-$root/runs}"
if [[ -d /data && -z "${RUNS_ROOT:-}" ]]; then base=/data/coffee-shelf-physical-service-runs; fi
out="$base/$mode/$(date -u +%Y%m%dT%H%M%S%N)"
mkdir -p "$out"
export TASK_TRACE="$out/trace.jsonl"
export COFFEE_CONTROL_TRACE="$out/control.jsonl"
python3 evidence.py snapshot "$out/sources.json"
inspect-robots run --embodiment coffee_yam --policy "$policy" \
  -E "task=$root/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width="$width" -E cam_height="$height" \
  --instruction 'Make one coffee and return the filled mug to its own saucer. Leave the machine and both mugs released.' \
  --scorer task_score --max-steps "$steps" --epochs 1 --seed 0 --store-frames --log-dir "$out"
