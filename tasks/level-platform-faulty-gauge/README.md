# Calibrate an angle gauge and level a platform

Category: Impact on jobs. The task represents industrial machine-bed levelling with a biased instrument. Measure the gauge on a level reference, measure the platform, and operate physical buttons until true tilt is within 0.15 degrees.

## Scope and setup

The official bimanual YAM station uses parallel grippers. Coordinates are metres in the station frame, with worktop height 0.7475 m. See task.py for exact positions. The fixed reference block is 90 x 80 x 50 mm. The movable gauge is a 50 mm cube, mass 160 g. Two 35 x 35 mm button fixtures have 28 x 28 x 12 mm caps and 8 mm spring travel. The hinged platform is 200 x 140 mm. Initial tilt is 1.5 degrees and gauge offset is 0.8 degrees. Each button edge changes the motor target by 0.25 degrees.

A separate 92 x 34 mm public display shows the gauge reading with four colored corner fiducials. This fixed-layout benchmark models rigid contact, spring buttons and a motorised platform. It does not randomise gauge offset or certify real instrument accuracy. Geometry, materials, motor settings and scoring are fully defined in task.py and review_native_common.py.

## Run and score

Use the existing official Isaac Lab Arena / Inspect Robots YAM environment. Install runtime and policy into the same isolated Python environment as the CLI.

```bash
uv pip install --python "$VIRTUAL_ENV/bin/python" --no-deps -e runtime -e policy
export PYTHONPATH="$PWD/runtime:$PWD/policy:$PWD:${PYTHONPATH:-}"
export YAM_TELEMETRY="$PWD/demo/trajectory.jsonl"
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment pr71_yam --policy pr71_visible -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=720 --epochs 1 --seed 0 --max-steps 2400 --instruction "$instruction" --scorer task_score --store-frames --log-dir "$PWD/demo"
```

For Dream-1, set DREAM_API_KEY in the environment, use policy dream and separate telemetry/log directories, retaining the 2,400-step limit. The scripted policy reads RGB digits, public time and nominal geometry. Only robot joint commands move the gauge. Reset restores initial fixture coordinates and motor targets before settling.

Scoring gives 0.25 for a stable reference reading, 0.5 for a stable platform reading, and 0.75 after both. A reading requires one stationary second with hands clear. Full score additionally requires true tilt within 0.15 degrees and hands clear for one second. Multiply the current stage by max(0, 1 - 0.1*(total presses - abs(net presses)) - 0.1*max(0,total presses-12)). Score uses final state. Exact pose, velocity and clearance thresholds are in task.py.

## Selected evidence

The scripted RGB-feedback trial scored 1.0 in 1,140 steps using six presses. Dream-1 scored 0.0 over 2,400 steps. Simulated durations are 57 and 120 seconds. Wall time also includes renderer startup and network inference. Dream SDK expired-session retries are disclosed in the retained runtime log.

All three sensors in both trials produced native 1280 x 720 RGB frames. Six H.264 exports have 20 fps, 1,141 demo frames and 2,401 model frames. Each is under 50 MB. The local package contains trial JSON, screenshots and recordings. Its submit.py uploads the overview videos and setup screenshot when explicitly run by the user.

Development draft metadata and evidence.py predate these runs. The selected trial records, task.json, VALIDATION.json and SHA256.json describe this package.

## Importance evidence

Reference-check an angle gauge before adjusting equipment level. See [IMPORTANCE.md](IMPORTANCE.md) for primary sources, the category rationale and remaining review risks.
