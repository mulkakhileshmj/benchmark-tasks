# Index a Geneva drive and prove its dwell

An intermittent mechanism must advance only during engagement and remain still between engagements. This trainer checks two complete indexing cycles.

## Setup

A free steel training fixture sits at x 0.23 and y 0 on the official YAM table. The base is 290 by 180 by 8 mm with a rear support tab. Its base mass is 3.2 kg. Two vertical shafts are 80 mm apart. The blue driver has a pin at radius 56.569 mm. The orange four slot star has 60 mm outer radius. Both wheel plates are 12 mm thick. The pin is 8 mm diameter. Radial slot half width is 5.5 mm. The lock radius is 43 mm with 44.5 mm cutouts. A freely swivelling blue grip sits above the pin. All shaft angles start at zero. Both arms begin in the official ready pose. Exact convex pieces and component offsets are in geometry.py and task.py.

## Reproduce


Use the official YAM launchable with Arena release 0.3.1 and YAM revision `0059d02a1a91bc6834c1d0846022891df0c78730`.

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e runtime -e policy
TASK_INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 inspect-robots run \
  --embodiment yam_articulated --policy yam_feedback \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=720 --seed 0 --epochs 1 \
  --instruction "$TASK_INSTRUCTION" --scorer task_score --max-steps 900 \
  --store-frames --log-dir runs/demo
```

For the model use `dream_current` with `-P server_url=http://127.0.0.1:8241` and `--max-steps 2400` in a separate log directory. The checkpoint is `dreamlabs/dream-1-beta`. Install `dreamlabs==1.0.37` and `dreamlabs-client==1.1.3` in the bridge environment. Start it with `DREAM_BRIDGE_PORT=8241 JAX_PLATFORMS=cpu python policy/current_api_bridge.py`. Keep credentials outside the repository.

Run one Dream episode at a time. A provider session replacement resets model memory but does not reset the scene. The bridge records replacements in `dream_session.json` and allows at most two. A failed evaluation is not a score. There is no scripted fallback. The scripted demo is a separate feasibility trial.

The model limit is 120 seconds of control time. The demo limit is 45 seconds. Wall time depends on inference and rendering. Full success may finish early. Seed 0 and zero object jitter reproduce the authored layout. Start a new process to reset the scene and score history before every trial.

Render the completed trial with `inspect-robots video LOG.json`. Preserve overhead and both wrist views at native 1280 by 720. Verify frame counts equal control steps plus one. The selected hashes and receipts are recorded in `validation.json` and `video_audit.json`.


## Scoring

Zero after direct robot contact with the output or fixture translation above 5 mm or fixture rotation above 5 degrees or backward input motion below minus 5 degrees or output movement above 5 degrees during a dwell. Score 0.25 after one correct quarter turn with input beyond 300 degrees and output within 5 degrees of minus 90 degrees. Score 0.5 after the second quarter turn with input beyond 650 degrees and output within 5 degrees of minus 180 degrees. Score 0.75 when input is within 8 degrees of 720 degrees and output is within 5 degrees of minus 180 degrees and both angular speeds are below 0.05 rad/s. Score 1 when these final conditions also hold with both hands open and every robot contact clear. Full success must hold for one second.

## Fidelity

Native rigid contact between a roller pin and a convex approximation of a slotted star and locking profile. Neither shaft has a position drive. The base remains free. The steel base mass and small contact offsets stabilize the contact calculation. No object pose is set during the episode. The feasibility expert may read measured state. Dream receives only camera frames and robot state.

## Novelty and validation

PR 233 rotates a table to a lock pin. PR 828 locates TDC with a piston stop. This task instead uses native roller contact with a slotted star and a locking profile to prove two index events and dwell. A heavier free base prevents sliding without a world constraint. The successful demo retained the original scoring thresholds. The selected expert uses measured fixture pose only to correct its robot targets. It never writes a prop pose.

## Selected current trials

Dream 1 scored 0.0 in 2400 control steps with a limit of 2400 steps. The separate scripted demo scored 1.0 in 752 control steps. Both used the same task source and seed 0. Neither trial reported an evaluation error. These are single trials and do not estimate a general success rate.

The task source SHA256 is `c542466397287ebf45b932d3e1387984d5ba28b434a49e5164e1ee7b755218cd`. validation.json identifies the selected receipts. video_audit.json records the checksums and native camera frame counts. The score plot shows measured scores through the recorded trajectories.

This is a new task idea. The novelty comparison is recorded above.

The model followed the official SDK recovery contract. There were 0 provider session replacements. Each replacement resets model memory and is recorded in validation.json. The physical scene was never reset during the run. No fallback actions were used.

## Mechanism counterexample

Removing the roller pin while retaining the same robot expert and scoring gives 0.0. The measured receipt is in counterexample.json. The production demo reaches 1.0. Both keep the same dwell and fixture movement requirements. The standalone geometry validator allows the refined convex tessellation. Its maintenance is recorded separately from the unchanged runtime source hashes in validation.json.
