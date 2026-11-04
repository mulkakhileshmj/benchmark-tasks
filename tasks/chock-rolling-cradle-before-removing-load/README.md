Chock a rolling cradle before removing its load

A rolling cradle can shift while a robot removes its load. Each arm first places a chock on one side of the cradle. Only after both chocks are seated may an arm lift the load and place it on the white storage pad. The equipment is unpowered.

The scripted demonstration reaches 1.0 in two runs. Both uploaded videos show the same scripted demonstration. No model score is claimed. The cradle has a fixed internal ballast. Use the official RoboCurve YAM launchable. Exact installed dependency versions and the source hash are in validation.json. Every solid and initial pose is in scene_inventory.json. Object dimensions are metres. Masses are kilograms. The code in task.py implements the numerical scoring conditions in task.json.

```bash
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python -e tasks/chock-rolling-cradle-before-removing-load/runtime
source /data/IsaacLab-Arena/yam.env
inspect-robots run --embodiment yam_articulated --policy yam_scripted \
  -E task=tasks/chock-rolling-cradle-before-removing-load/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=1280 -E cam_height=720 \
  --seed 0 --epochs 1 --instruction "Place one blue chock on each side of the grey rolling cradle before removing its orange load. Put the load on the white storage pad. Leave both chocks against the cradle. Release both grippers. Keep the cradle within 10 mm of its starting position. The cradle has a fixed internal ballast. All equipment is unpowered." \
  --scorer task_score --max-steps 500 --store-frames --log-dir runs/demo
```

Run one epoch per process. The successful L40S trial took about 150 seconds. The camera videos are native 1280 by 720 at 20 frames per second. Full success includes two seconds of the released state. Reset to the listed starting layout before each trial.
