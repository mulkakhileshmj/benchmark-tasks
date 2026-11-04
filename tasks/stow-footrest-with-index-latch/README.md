Stow a footrest with its index latch

Clearing an empty footrest from a transfer space is useful when preparing an assistive device. The training module requires a separate hand to withdraw its latch before the footrest can swing. No person or occupant is present.

The scripted demonstration reaches 1.0. No model score is claimed. Use the official RoboCurve YAM launchable. Exact installed dependency versions and the source hash are in validation.json. Every solid and initial pose is in scene_inventory.json. Object dimensions are metres. Masses are kilograms. The code in task.py implements the numerical scoring conditions in task.json.

```bash
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python -e tasks/stow-footrest-with-index-latch/runtime
source /data/IsaacLab-Arena/yam.env
inspect-robots run --embodiment yam_articulated --policy yam_scripted \
  -E task=tasks/stow-footrest-with-index-latch/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=1280 -E cam_height=720 \
  --seed 0 --epochs 1 --instruction "Stow the footrest module so the marked transfer space is clear. Pull the grey latch back and hold it while swinging the blue footrest through 90 degrees. Return the latch into the second index slot. Release both handles. The module is unoccupied and unpowered." \
  --scorer task_score --max-steps 600 --store-frames --log-dir runs/demo
```

Run one epoch per process. The successful L40S trial took about 269 seconds while three scenes ran concurrently. The camera videos are native 1280 by 720 at 20 frames per second. Full success includes two seconds of the released state. Reset to the listed starting layout before each trial.

Both uploaded video fields contain the same scripted demonstration. No model execution or model score is claimed. The video combines synchronized top and wrist camera frames.
