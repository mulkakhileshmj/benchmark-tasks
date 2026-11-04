Assemble the two spacer halves around the captive shaft. The wide head blocks placing a complete ring from above. The full score demo uses the scripted policy. The model run is unscored.

Use the official RoboCurve YAM launchable. The task uses Python 3.12 and Isaac Lab Arena 0.3.1. Runtime versions and the source hash are in validation.json. Every solid and starting pose is in scene_inventory.json. Dimensions are metres. Masses are kilograms. The scoring function in task.py implements the numeric conditions in task.json.

Install the local runtime in the launchable environment.

```bash
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python -e tasks/assemble-split-shaft-spacer/runtime
source /data/IsaacLab-Arena/yam.env
```

Run the full score demonstration. Scoring is recorded in the JSON trial log. Use one epoch per process.

```bash
inspect-robots run --embodiment yam_articulated --policy yam_scripted \
  -E task=tasks/assemble-split-shaft-spacer/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=1280 -E cam_height=720 \
  --seed 0 --epochs 1 --instruction "Fit both blue halves of the spacer collar around the captive square shaft below its wide head. Bring the halves together from opposite sides. Leave both seams closed to within 4 mm. Release both halves and keep the grey shaft fixed. The fixture is unpowered." \
  --scorer task_score --max-steps 600 --store-frames --log-dir runs/spacer_demo
```

The L40S demo took about 118 seconds with three scenes running concurrently after shader compilation. The native videos are 1280 by 720 at 20 frames per second. The full score demonstration includes two seconds of stable released enclosure. Open fingers must have measured travel beyond 35 mm. Restore the source layout to reset the task.

The operation differs from separating a snap case because both halves must close around a third captive object. It differs from a spring cage fastener because no leg is compressed and no hook passes through a plate. It also differs from a slotted partition because the result must enclose an inaccessible shaft.
