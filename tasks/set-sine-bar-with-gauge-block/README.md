# Set a sine bar angle using one gauge block

The roller spacing is 130 mm. A 65 mm rise gives 30 degrees because the rise is half that spacing. The robot must lift the bar far enough for the other hand to insert the block. It then lowers the bar and releases it. The scored support force comes from native contact physics.

The initial low lift did not leave room for the gripper. The final demonstration lifts the bar to 70 degrees before insertion and lowers it onto the support. The bar is never assigned a pose during a trial. Its hinge has zero drive force.

## Reproduce

Use the official YAM launchable with Arena release 0.3.1 and YAM revision `0059d02a1a91bc6834c1d0846022891df0c78730`.

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

For the genuine model run use `dream_current` with `-P server_url=http://127.0.0.1:8241` and `--max-steps 2400` and a separate log directory. Use `dreamlabs/dream-1-beta`. Install `dreamlabs==1.0.37` and `dreamlabs-client==1.1.3` in the separate bridge environment. Start it with `DREAM_BRIDGE_PORT=8241 JAX_PLATFORMS=cpu python policy/current_api_bridge.py`. Keep credentials outside this repository.

The bridge follows the documented SDK continuation contract after a provider session replacement. Such a replacement resets model memory. Every event is recorded in dream_session.json and validation.json. The scene remains paused while the request recovers. More than two replacements fails the trial. No fallback action is substituted.

Render a completed log with `inspect-robots video LOG.json`. The selected native cameras are 1280 by 720 at 20 frames per second. Separate model and demo receipts and video frame counts and source hashes are recorded in validation.json and video_audit.json.

## Proof and limits

Full credit requires measured vertical force above 0.02 N from the bar onto the blue block. An aligned bar still held by a robot does not qualify. A block stacked above another block fails the floor height test. The released demo produced about 0.91 N of vertical contact force. The detailed force matrix is used because the backend scalar contact data buffer reports zero force for these contacts.

## Novelty comparison

PR 233 rotates an off balance table and locks it with a pin. Our earlier footrest task changes an indexed hinge position while retracting a spring pin. This task selects a support height and sets an angle through the sine relation and actual weight bearing contact. There is no index pin or final powered angle target. No sine bar or gauge block task appeared in the reviewed PR titles and descriptions. This is not a guarantee of acceptance.

## Selected current trials

Dream 1 scored 0.0 in 2400 control steps with a limit of 2400 steps. The separate scripted demo scored 1.0 in 436 control steps. Both used the same task source and seed 0. Neither trial reported an evaluation error. These are single trials and do not estimate a general success rate.

The task source SHA256 is `184bac2986179a64e9ff7ef35dd07456d74f524fe322d639e7cce99c42d51aef`. validation.json identifies the selected receipts. video_audit.json records the checksums and native camera frame counts. The score plot shows measured scores through the recorded trajectories.

This is a new task idea. The novelty comparison is recorded above.

The model followed the official SDK recovery contract. There were 2 provider session replacements. Each replacement resets model memory and is recorded in validation.json. The physical scene was never reset during the run. No fallback actions were used.
