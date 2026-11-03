# Release a loaded spring platform under control

The bar physically restrains the spring loaded platform. One arm grips the separate support tab while the other withdraws the bar. The supporting arm then raises the platform slowly to its unloaded equilibrium. The bar is returned to its pad.

## Reproduce the trials

Use the official YAM Isaac Lab Arena environment. The recorded environment uses the YAM repository revision `0059d02a1a91bc6834c1d0846022891df0c78730` and Arena release 0.3.1.

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e runtime -e policy
OPENBLAS_NUM_THREADS=1 python validate.py
TASK_INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 MKL_NUM_THREADS=1 inspect-robots run \
  --embodiment yam_articulated --policy yam_scripted \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=720 --seed 0 --epochs 1 \
  --instruction "$TASK_INSTRUCTION" --scorer task_score --max-steps 1200 \
  --store-frames --log-dir runs/demo
```

For the model trial use the current official Dream SDK in a separate environment. This keeps its IK dependencies separate from the simulator. Configure the private credential file using the Dream Labs setup guide. No credential belongs in this repository.

```bash
uv venv --python 3.12 .dream-env
uv pip install --python .dream-env/bin/python dreamlabs==1.0.23
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 JAX_PLATFORMS=cpu .dream-env/bin/python policy/current_api_bridge.py
```

Keep that bridge running in a separate terminal. Run the same evaluation command with `--policy dream_current` and `-P model=dreamlabs/dream-1-beta` and `-P server_url=http://127.0.0.1:8240`. Give it a separate log directory. The bridge uses the documented video names and arm state fields with client IK. It retries session expiry using the SDK recovery contract. It forwards actual model action chunks. The normal embodiment bounds and motion guardrails remain enabled.

Render a completed log with `inspect-robots video LOG.json`. Published media is H264 at 1280 by 720. A detail inset uses a synchronized recorded camera. It does not change the timeline. validation.json records the selected trial scores and durations and source revisions.

## Geometry and reset

scene_inventory.json lists all local cuboids and link poses and joint settings. Compound size values are validation hints. The local collider solids and joint poses define the physical geometry. Use rigid plastic training props with ballast to match the stated masses. Fix the static fixture roots to the table. Follow the reset procedure in task.json. No powered equipment is involved.

## Novelty review

The live board contains speed constrained carrying and mechanical restraint tasks. This task grades the release of stored mechanical energy during removal of the restraint. The moving platform is driven by a passive physical spring rather than an abstract event flag.

## Recorded evaluation

The scripted demo scored 1.0 in 337 control steps. Its wall runtime was 118.53 seconds. The Dream 1 run scored 0.0 in 1200 control steps. Its wall runtime was 534.16 seconds. Both trials used seed 0 and the same task source. These are single trials. The model checkpoint was `dreamlabs/dream-1-beta`. The videos include every recorded frame through the final state. See validation.json and video_audit.json for the source revisions and media checks.
