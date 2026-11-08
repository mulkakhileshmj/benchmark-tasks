# Find polarizer extinction and power down

Optical bench setup requires distinguishing a real dark orientation from a source that is simply switched off.

## Setup

Use the official bimanual YAM table. Coordinates are metres in the station frame. A fixed grey aluminium training base is centred at x 0.19 and y 0. Its main plate is 250 by 280 by 8 mm. A blue 80 g analyzer disc is 90 mm diameter and 8 mm thick on a passive vertical axis 50 mm above the table. A thin 8 mm diameter post carries a freely swivelling blue metal grip at radius 60 mm. The grip is 22 mm diameter and 20 mm high with mass 25 g and centre 110 mm above the table. The green polymer source cap is 26 by 26 by 20 mm with mass 20 g at x 0.16 and y 0.20 and centre 30 mm above the table. It moves down 12 mm against a 200 N/m spring. Its support pad is 40 by 40 by 10 mm. A 60 by 30 by 25 mm detector patch shows normalized transmission through green brightness. The analyzer starts at zero degrees with the source off. Its allowed travel is zero to 180 degrees. The fixed filter axis is 35 degrees. Exact post and patch offsets are in task.py. All colliders use the official simulator contact material rather than calibrated optical hardware materials.

The selected scripted trial used `OPTICAL_DEMO_DARK_DEGREES=110`. A separate trial at 107 degrees also scored 1.0. Set this environment variable before the scripted command below to reproduce the selected trial.

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


## Scoring

Score 0.25 after powered normalized intensity exceeds 0.90 for 0.5 seconds with stable angle. After that bright proof score 0.75 after powered intensity stays below 0.01 for 0.5 seconds with stable angle. Stable means the measured angle spans less than 0.5 degrees over the preceding full 0.5 seconds. Thus a momentary bright or dark crossing never establishes a proof. Moving more than 2 degrees from the last qualified dark setting clears that proof. Score 1 when both proofs hold and the source is off and the button returns above minus 3 mm and the angle spans less than 0.25 degrees over the preceding full 0.5 seconds and both grippers are open and every robot contact is clear. Otherwise score 0. A powered off source never establishes extinction. Full success must hold for one second.

## Fidelity

Native rigid contact and passive analyzer and source button joints. An ideal Malus law signal equals cosine squared of analyzer angle minus 35 degrees while powered. It does not simulate optical waves or detector noise. The detector color is a visual readout and never changes a physical pose. There is no analyzer angle servo. Stillness uses measured pose history rather than backend instantaneous velocity because contact can produce high frequency solver velocity noise. Optical intensity and power sequence gates are unchanged.

## Novelty and validation

PR 996 calibrates a reference and measures cuvette absorbance. PR 1045 places edging blocks on lenses. This task instead finds crossed polarizer extinction through analyzer rotation while proving that the source is powered. The separate feasibility demo commands robot joints and never writes a prop pose. Dream receives camera frames and robot state.

## Selected current trials

Dream 1 scored 0.0 in 2400 control steps with a limit of 2400 steps. The separate scripted demo scored 1.0 in 576 control steps. Both used the same task source and seed 0. Neither trial reported an evaluation error. These are single trials and do not estimate a general success rate.

The task source SHA256 is `90bf69ecda1cd9878c6ba26e9595a2971087eb1b68ded754de33bba965cc4eb6`. validation.json identifies the selected receipts. video_audit.json records the checksums and native camera frame counts. The score plot shows measured scores through the recorded trajectories.

This is a new task idea. The novelty comparison is recorded above.

The model followed the official SDK recovery contract. There were 0 provider session replacements. Each replacement resets model memory and is recorded in validation.json. The physical scene was never reset during the run. No fallback actions were used.

Two additional native checks changed only demo actions. With no source press the score was 0.0. With powered brightness but no extinction the score was 0.25. The physics and scoring source before demo() was identical. counterexamples.json records both measured receipts. The compact proof archive on D retains the changed demo sources and full trajectories.
