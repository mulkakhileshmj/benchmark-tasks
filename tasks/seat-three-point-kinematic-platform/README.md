# Seat a platform on three kinematic datums

Instrument carriers need repeatable support without rocking. The task seats three ball feet on a cup and a groove and a flat and verifies that every support carries load.

The selected demo uses tall handle stems so the grippers clear the plate. The scorer is unchanged from the earlier unsuccessful grasp. Each of the three contacts must support load. A physical reproduction can use thin force sensors under the three datums plus a level and ruler. A photograph alone cannot establish the 0.02 N force threshold.

## Reproduce

Use the official YAM launchable with Arena release 0.3.1 and YAM revision `0059d02a1a91bc6834c1d0846022891df0c78730`.

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e runtime -e policy
TASK_INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 inspect-robots run \
  --embodiment yam_articulated --policy yam_scripted \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=720 --seed 0 --epochs 1 \
  --instruction "$TASK_INSTRUCTION" --scorer task_score --max-steps 650 \
  --store-frames --log-dir runs/demo
```

For the model use `dream_current` with `-P server_url=http://127.0.0.1:8241` and `--max-steps 2400` in a separate log directory. The checkpoint is `dreamlabs/dream-1-beta`. Install `dreamlabs==1.0.37` and `dreamlabs-client==1.1.3` in the bridge environment. Start it with `DREAM_BRIDGE_PORT=8241 JAX_PLATFORMS=cpu python policy/current_api_bridge.py`. Keep credentials outside the repository.

Run one Dream episode at a time. A provider session replacement resets model memory but does not reset the scene. The bridge records replacements in `dream_session.json` and allows at most two. A failed evaluation is not a score. There is no scripted fallback. The scripted demo is a separate feasibility trial.

The model limit is 120 seconds of control time. The demo limit is 32.5 seconds. Wall time depends on inference and rendering. Full success may finish early. Seed 0 and zero object jitter reproduce the authored layout. Start a new process to reset the scene and score history before every trial.

Render the completed trial with `inspect-robots video LOG.json`. Preserve overhead and both wrist views at native 1280 by 720. Verify frame counts equal control steps plus one. The selected hashes and receipts are recorded in `validation.json` and `video_audit.json`.

## Scoring and fidelity

Score 0 if a datum moves more than 2 mm or the plate passes more than 8 mm below the table. Score 1 when plate x and y errors are each below 3 mm and height error below 3 mm and tilt and yaw below 3 degrees and all three vertical support forces exceed 0.02 N and plate speed is below 8 mm/s and angular speed below 0.1 rad/s and both grippers are open and clear. The target plate centre is x 0.26 and y 0 and 42.142 mm above the table. Score 0.75 for the same seated state before robot release. Score 0.5 when at least one support carries above 0.02 N and tilt is below 10 degrees. Score 0.25 when the lowest plate point is more than 15 mm above the table. Otherwise score 0. Full success must hold for one second.

Native rigid body contact between sphere feet and convex cup and groove faces and a flat support. Contact force matrices supply each support force. Robot contacts must be clear at full score. The cup is a square cone approximation. Surface compliance and manufacturing tolerances are not simulated. Datums are fixed. The plate has no pose servo and is not teleported during a trial.

## Novelty review

The task requires a cone and V groove and flat to constrain and support a freely handled plate. PR 59 places ballast on a fixed deck and calculates its mass centre. PR 437 balances and loads a bar on a post. This task instead verifies three separate native contact forces and seating alignment after both hands release.

The live title inventory was refreshed on 8 November 2026. Source comparisons and the exact inventory are preserved with the local audit. Receipt is not acceptance.

## Selected current trials

Dream 1 scored 0.0 in 2400 control steps with a limit of 2400 steps. The separate scripted demo scored 1.0 in 242 control steps. Both used the same task source and seed 0. Neither trial reported an evaluation error. These are single trials and do not estimate a general success rate.

The task source SHA256 is `a89aaa63a77b17793d5999ed850b401eaa0e808a072a0c5f4f9f68da09b54e3c`. validation.json identifies the selected receipts. video_audit.json records the checksums and native camera frame counts. The score plot shows measured scores through the recorded trajectories.

This is a new task idea. The novelty comparison is recorded above.

The model followed the official SDK recovery contract. There were 0 provider session replacements. Each replacement resets model memory and is recorded in validation.json. The physical scene was never reset during the run. No fallback actions were used.
