# Set a differential angle while keeping its output centred

Both input forks touch one bar on a floating central carriage. Opposite input motion rotates the bar while equal input motion moves the carriage. The output comes from native contact physics. No task code computes and assigns a carriage pose.

The centre band applies throughout the movement. A sequential adjustment that first displaces the carriage cannot recover credit by returning it later. Direct robot contact with either output body invalidates the trial.

## Reproduce

Use the official bimanual YAM launchable with Arena release 0.3.1 and YAM revision `0059d02a1a91bc6834c1d0846022891df0c78730`.

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e runtime -e policy
OPENBLAS_NUM_THREADS=1 python validate.py
TASK_INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 inspect-robots run \
  --embodiment yam_articulated --policy yam_scripted \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=720 --seed 0 --epochs 1 \
  --instruction "$TASK_INSTRUCTION" --scorer task_score --max-steps 650 \
  --store-frames --log-dir runs/demo
```

The real model uses `dreamlabs/dream-1-beta`. Install `dreamlabs==1.0.37` and `dreamlabs-client==1.1.3` in a separate bridge environment. Keep its credential outside this repository. Start the supplied bridge with `DREAM_BRIDGE_PORT=8241 JAX_PLATFORMS=cpu python policy/current_api_bridge.py`. Use the same run command with `--policy dream_current` and `-P server_url=http://127.0.0.1:8241` and `--max-steps 2400`. Give it a separate log directory. The SDK may adopt a replacement session after expiry. This resets model memory. The bridge follows the documented continuation contract while the simulator is paused. Every replacement is recorded in dream_session.json and validation.json. More than two replacements fails the trial. There is no scripted fallback.

Render a completed log with `inspect-robots video LOG.json`. The selected recordings use native 1280 by 720 cameras at 20 frames per second. Model and demo evidence are separate. validation.json records the exact measured scores and source revision after validation finishes.

## Causal check

Run the demo command with `-E task="$PWD/one_input_only.py"` in a separate log directory. This holds the left input still while moving the right input. It must score zero. The simultaneous demo must score 1.0. The source and joint traces distinguish an actual coupled movement from independent decorative handles.

## Novelty comparison

PR 437 balances a loose bar on a post while adding masses. This task has a guided carriage and two independent constrained input forks. It changes angle while limiting translation throughout the motion. PR 233 directly rotates an off balance table and inserts an index pin. This task has no locking pin. It requires continuous coordination through a floating output. The idea is absent from our original 13 submissions. The comparison does not guarantee an acceptance decision.

## Selected current trials

Dream 1 scored 0.0 in 2400 control steps with a limit of 2400 steps. The separate scripted demo scored 1.0 in 232 control steps. Both used the same task source and seed 0. Neither trial reported an evaluation error. These are single trials and do not estimate a general success rate.

The task source SHA256 is `8c14400dc5b5a54f3846c97467277797485607df066d6f06d4d1ff700e87dab5`. validation.json identifies the selected receipts. video_audit.json records the checksums and native camera frame counts. The score plot shows measured scores through the recorded trajectories.

This is a new task idea. The novelty comparison is recorded above.

The model followed the official SDK recovery contract. There were 0 provider session replacements. Each replacement resets model memory and is recorded in validation.json. The physical scene was never reset during the run. No fallback actions were used.
