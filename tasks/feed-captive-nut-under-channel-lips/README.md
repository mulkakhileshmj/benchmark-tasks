Feed a captive nut under channel lips

A fastener in an enclosed channel must enter through the open end. Its wide head must finish under both retaining lips. This tests a constrained assembly path while the other arm braces the loose channel.

The scripted demonstration reaches 1.0. No model score is claimed. Use the official RoboCurve YAM launchable. Exact installed dependency versions and the source hash are in validation.json. Every solid and initial pose is in scene_inventory.json. Object dimensions are metres. Masses are kilograms. The code in task.py implements the numerical scoring conditions in task.json.

```bash
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python -e tasks/feed-captive-nut-under-channel-lips/runtime
source /data/IsaacLab-Arena/yam.env
inspect-robots run --embodiment yam_articulated --policy yam_scripted \
  -E task=tasks/feed-captive-nut-under-channel-lips/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=1280 -E cam_height=720 \
  --seed 0 --epochs 1 --instruction "Hold the grey channel still. Feed the blue T nut through its open front end and slide it to the white end mark. The wide head must be below both retaining lips. Release both grippers. Keep the channel within 5 mm of its starting position. All equipment is unpowered." \
  --scorer task_score --max-steps 600 --store-frames --log-dir runs/demo
```

Run one epoch per process. The successful L40S trial took about 153 seconds while three scenes ran concurrently. The camera videos are native 1280 by 720 at 20 frames per second. Full success includes two seconds of the released state. Reset to the listed starting layout before each trial.

Both uploaded video fields contain the same scripted demonstration. No model execution or model score is claimed for this submission.
