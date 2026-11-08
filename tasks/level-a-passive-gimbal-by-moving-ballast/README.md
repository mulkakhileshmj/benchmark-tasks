# Level a passive gimbal by moving ballast

An instrument platform should return to level after a disturbance without relying on an angle motor or a brake that remains applied.

## Setup

Use the official bimanual YAM table. Coordinates are metres in the station frame. A grey fixed metal stand has a 300 by 300 by 8 mm base. The two passive axes meet at x 0.27 and y 0 and 70 mm above the table. An 80 g roll frame is 210 mm square with 10 mm rails. A blue 200 g platform is 160 by 160 by 8 mm. Its authored mass centre is 25 mm forward and 15 mm left and 12 mm below the pivot. Two captive 100 g metal ballast blocks are each 30 by 30 by 16 mm with raised polymer grips on thin stems. Each grip is 26 by 26 by 22 mm and its centre is 60 mm above its ballast body frame. The x ballast starts 40 mm forward and 40 mm right and 8 mm below the platform frame. It travels 65 mm along x. The y ballast starts 40 mm behind and 40 mm left and 8 mm below the platform frame. It travels 50 mm along y. The grey brake caps are 24 mm cubes with mass 30 g. They start 110 mm behind the pivot and 200 mm to either side and 80 mm above the table. Each travels 20 mm along x on an ideal native guide attached to the fixed stand. Both brakes start closed. Both platform axes start at zero with limits of plus or minus 20 degrees. The source declares every primitive and mass centre.

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
  --instruction "$TASK_INSTRUCTION" --scorer task_score --max-steps 1600 \
  --store-frames --log-dir runs/demo
```

For the model use `dream_current` with `-P server_url=http://127.0.0.1:8241` and `--max-steps 2400` in a separate log directory. The checkpoint is `dreamlabs/dream-1-beta`. Install `dreamlabs==1.0.37` and `dreamlabs-client==1.1.3` in the bridge environment. Start it with `DREAM_BRIDGE_PORT=8241 JAX_PLATFORMS=cpu python policy/current_api_bridge.py`. Keep credentials outside the repository.

Run one Dream episode at a time. A provider session replacement resets model memory but does not reset the scene. The bridge records replacements in `dream_session.json` and allows at most two. A failed evaluation is not a score. There is no scripted fallback. The scripted demo is a separate feasibility trial.

The model limit is 120 seconds of control time. The demo limit is 80 seconds. Wall time depends on inference and rendering. Full success may finish early. Seed 0 and zero object jitter reproduce the authored layout. Start a new process to reset the scene and score history before every trial.

Render the completed trial with `inspect-robots video LOG.json`. Preserve overhead and both wrist views at native 1280 by 720. Verify frame counts equal control steps plus one. The selected hashes and receipts are recorded in `validation.json` and `video_audit.json`.


The two disturbance caps have centres 80 mm above the pivot and 140 mm away along positive y and negative x. Their outboard stems keep the ballast withdrawal paths clear. task.py declares every primitive.

## Scoring

Both brake sliders must exceed 15 mm to free the platform. Record the two ballast positions on release. Reapplying either brake or changing either ballast more than 0.5 mm clears all proofs. First show level within 2 degrees on both axes with angular speeds below 0.04 rad/s and robot contact clear for one second. Then disturb roll beyond 8 degrees and show the same free return to level. Then disturb pitch beyond 8 degrees and show another free return to level. Each proof earns 0.25. All three proofs earn 1 when the platform remains free and clear and both grippers are open with the final one second hold and all robot contacts clear. Otherwise retain 0.25 per completed proof.

## Fidelity

Native articulated gravity dynamics with authored mass centres and passive damping and joint friction. Ballast mass centres remain at the dense blocks and simplify the mass of the raised control stems. Native guide constraints attach the outboard brake controls to the stand without rendering detailed brackets. The brakes change native velocity damping and effort limits. Target velocity is zero and angular stiffness is zero. There is no angle target or levelling motor. Ballast joints have passive force limited dampers. The feasibility expert reads measured joints and commands only the robot arms. Dream receives cameras and robot state only. Surface compliance and bearing wear are not simulated. Internal articulation collisions are disabled. Raised control stems simplify the mass distribution. The disturbance caps remain native rigid robot contact surfaces.

## Novelty and validation

PR 59 computes static mass centre on a fixed deck. PR 71 adjusts a powered foot using a faulty gauge. PR 944 uses a gimbal as a tong pivot. Here the robot moves captive ballast and releases both passive brakes then demonstrates gravity return after separate disturbances. No position servo supplies that return.

## Selected current trials

Dream 1 scored 0.0 in 2400 control steps with a limit of 2400 steps. The separate scripted demo scored 1.0 in 1023 control steps. Both used the same task source and seed 0. Neither trial reported an evaluation error. These are single trials and do not estimate a general success rate.

The task source SHA256 is `0ab95a558237a6a0ce02eea57fdade5be8061cfe3be356127acb2c8a9662facc`. validation.json identifies the selected receipts. video_audit.json records the checksums and native camera frame counts. The score plot shows measured scores through the recorded trajectories.

This is a new task idea. The novelty comparison is recorded above.

The model followed the official SDK recovery contract. There were 0 provider session replacements. Each replacement resets model memory and is recorded in validation.json. The physical scene was never reset during the run. No fallback actions were used.

Two additional native checks changed only expert actions. Leaving both brakes closed scored 0.0. Leaving the ballast unadjusted also scored 0.0. The scene and scorer source before demo() was identical. counterexamples.json records the measured receipts. The compact native proof archive on D retains both expert sources and full trajectories.
