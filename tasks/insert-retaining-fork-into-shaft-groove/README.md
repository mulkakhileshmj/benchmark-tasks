Insert a retaining fork into a shaft groove

Captive shafts need a retaining part inserted from the side below a head. One arm stabilises the loose fixture while the other feeds a rigid fork into a narrow axial groove. Both tines must pass the shaft centre.

The scripted demonstration reaches 1.0 in two runs. Both uploaded videos show the same scripted demonstration. No model score is claimed. Use the official RoboCurve YAM launchable. Exact installed dependency versions and the source hash are in validation.json. Every solid and initial pose is in scene_inventory.json. Object dimensions are metres. Masses are kilograms. The code in task.py implements the numerical scoring conditions in task.json.

```bash
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python -e tasks/insert-retaining-fork-into-shaft-groove/runtime
source /data/IsaacLab-Arena/yam.env
inspect-robots run --embodiment yam_articulated --policy yam_scripted \
  -E task=tasks/insert-retaining-fork-into-shaft-groove/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=1280 -E cam_height=720 \
  --seed 0 --epochs 1 --instruction "Hold the grey shaft fixture still with one arm. With the other arm slide the blue retaining fork sideways into the shaft groove below its square head. Both tines must reach beyond the shaft centre. Release both grippers. Keep the fixture within 5 mm of its starting position. All equipment is unpowered." \
  --scorer task_score --max-steps 500 --store-frames --log-dir runs/demo
```

Run one epoch per process. The successful L40S trial took about 92 seconds. The camera videos are native 1280 by 720 at 20 frames per second. Full success includes two seconds of the released state. Reset to the listed starting layout before each trial.
