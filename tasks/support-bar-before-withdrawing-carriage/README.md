Support a bar before withdrawing its carriage

A support must not be removed from beneath a precision part until another support carries the load. This task checks the order and continuous control during a two arm transfer. The optical bar is represented by a printed training part.

The scripted demonstration reaches 1.0. No model score is claimed. Use the official RoboCurve YAM launchable. Exact installed dependency versions and the source hash are in validation.json. Every solid and initial pose is in scene_inventory.json. Object dimensions are metres. Masses are kilograms. The code in task.py implements the numerical scoring conditions in task.json.

```bash
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python -e tasks/support-bar-before-withdrawing-carriage/runtime
source /data/IsaacLab-Arena/yam.env
inspect-robots run --embodiment yam_articulated --policy yam_scripted \
  -E task=tasks/support-bar-before-withdrawing-carriage/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=1280 -E cam_height=720 \
  --seed 0 --epochs 1 --instruction "Hold the blue optical bar level with the left arm before pulling its grey support carriage out with the right arm. Keep the bar supported while the carriage clears it. Lower the bar into the white storage cradle. Release both grippers. All equipment is unpowered and no person is present." \
  --scorer task_score --max-steps 600 --store-frames --log-dir runs/demo
```

Run one epoch per process. The successful L40S trial took about 247 seconds while three scenes ran concurrently. The camera videos are native 1280 by 720 at 20 frames per second. Full success includes two seconds of the released state. Reset to the listed starting layout before each trial.

Both uploaded video fields contain the same scripted demonstration. No model execution or model score is claimed. The video combines synchronized top and wrist camera frames.
