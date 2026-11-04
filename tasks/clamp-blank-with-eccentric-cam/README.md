Clamp a blank with an eccentric cam

Machine loading needs a blank to stay registered after the robot lets go. This task combines registration with a passive cam clamp rather than stopping at placement.

The scripted demonstration reaches 1.0. No model score is claimed. Use the official RoboCurve YAM launchable. Exact installed dependency versions and the source hash are in validation.json. Every solid and initial pose is in scene_inventory.json. Object dimensions are metres. Masses are kilograms. The code in task.py implements the numerical scoring conditions in task.json.

```bash
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python -e tasks/clamp-blank-with-eccentric-cam/runtime
source /data/IsaacLab-Arena/yam.env
inspect-robots run --embodiment yam_articulated --policy yam_scripted \
  -E task=tasks/clamp-blank-with-eccentric-cam/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=1280 -E cam_height=720 \
  --seed 0 --epochs 1 --instruction "Place the orange blank flat against both grey fixture stops. Hold it there while turning the blue cam lever to its closed stop past centre. Release both grippers. Leave the blank clamped after release. The fixture is bolted down and unpowered." \
  --scorer task_score --max-steps 600 --store-frames --log-dir runs/demo
```

Run one epoch per process. The successful L40S trial took about 159 seconds while three scenes ran concurrently. The camera videos are native 1280 by 720 at 20 frames per second. Full success includes two seconds of the released state. Reset to the listed starting layout before each trial.

Both uploaded video fields contain the same scripted demonstration. No model execution or model score is claimed for this submission.
