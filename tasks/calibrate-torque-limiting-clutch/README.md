# Calibrate a torque limiting clutch against two proof loads

A torque limiter must transmit ordinary load and slip before a higher load can damage equipment. This trainer tests one setting against both loads.

The selected feasibility demonstration uses a 10.5 mm preload target. Thresholds were not relaxed to make the demo pass. One unchanged setting carries the light load and slips against the heavy load. The proof uses measured joint travel and measured contact exclusions. A physical reproduction can use angle marks and travel scales with a force limited unpowered clutch trainer.

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

Score 0 after robot contact with the output or either load block or invalid joint state. Otherwise score 1 when both proofs hold and both grippers are open and all robot contacts are clear and angular speeds are below 0.05 rad/s and linear speeds below 4 mm/s. Score 0.75 for both proofs before release. Score 0.5 for the light proof. Score 0.25 for light spring compression above 5 mm and input angle above 10 degrees. Otherwise score 0. The light proof requires ten consecutive samples with selector below 5 mm and input above 20 degrees and output above 18 degrees and slip below 3 degrees and light compression above 10 mm. Return input and output within 3 degrees of zero before heavy testing. The heavy proof requires twenty samples with selector above 28 mm and heavy compression above 2 mm. Output range must be below 2 degrees while input advances more than 7 degrees. Final input must exceed 40 degrees and output must be between 8 and 18.5 degrees and slip must exceed 20 degrees. Changing preload more than 0.5 mm clears both proofs. Full success must hold for one second.

Native articulated rigid bodies and collision driven spring loads. Clutch capacity is an ideal passive velocity damper with force limit 0.010 plus 4 times preload displacement in metres. Capacity ranges from 0.010 to 0.090 Nm. It is not a friction disc or thermal wear model. The preload changes the torque limit only. No output angle target or object teleport is used. Robot contact monitoring forbids direct output and load manipulation.

## Novelty review

This task calibrates a torque capacity window through two physical load tests at one unchanged setting. PR 456 is a bolt tightening sequence. PR 233 indexes a rotary table and locks it with a pin. Neither establishes transmitted load followed by measured slip under a heavier load.

The live title inventory was refreshed on 8 November 2026. Source comparisons and the exact inventory are preserved with the local audit. Receipt is not acceptance.

## Selected current trials

Dream 1 scored 0.0 in 2400 control steps with a limit of 2400 steps. The separate scripted demo scored 1.0 in 509 control steps. Both used the same task source and seed 0. Neither trial reported an evaluation error. These are single trials and do not estimate a general success rate.

The task source SHA256 is `ee67256eda65e746b14b35f56ed51157bea7fb32281429d221ddf0316de878e0`. validation.json identifies the selected receipts. video_audit.json records the checksums and native camera frame counts. The score plot shows measured scores through the recorded trajectories.

This is a new task idea. The novelty comparison is recorded above.

The model followed the official SDK recovery contract. There were 0 provider session replacements. Each replacement resets model memory and is recorded in validation.json. The physical scene was never reset during the run. No fallback actions were used.
