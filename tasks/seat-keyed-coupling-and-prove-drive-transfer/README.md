Seat a keyed coupling and prove drive transfer

Robot drive maintenance requires a coupling to transmit rotation after assembly. This task checks both seating over unequal drive dogs and actual motion of the lower rotor. Placing the coupling on top without transferring rotation cannot earn full credit.

The scripted demonstration reaches 1.0. No model score is claimed. Use the official RoboCurve YAM launchable. Exact installed dependency versions and the source hash are in validation.json. Every solid and initial pose is in scene_inventory.json. Object dimensions are metres. Masses are kilograms. The code in task.py implements the numerical scoring conditions in task.json.

```bash
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python -e tasks/seat-keyed-coupling-and-prove-drive-transfer/runtime
source /data/IsaacLab-Arena/yam.env
inspect-robots run --embodiment yam_articulated --policy yam_scripted \
  -E task=tasks/seat-keyed-coupling-and-prove-drive-transfer/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=1280 -E cam_height=720 \
  --seed 0 --epochs 1 --instruction "Hold the lower drive hub still by its long lever. Seat the orange coupling over both unequal drive dogs. Release the lower lever and turn the orange coupling through 90 degrees so the lower hub turns with it. Release both grippers. Leave the coupling seated. All equipment is unpowered." \
  --scorer task_score --max-steps 600 --store-frames --log-dir runs/demo
```

Run one epoch per process. The successful L40S trial took about 114 seconds. The camera videos are native 1280 by 720 at 20 frames per second. Full success includes two seconds of the released state. Reset to the listed starting layout before each trial.

Both uploaded video fields contain the same scripted demonstration. No model execution or model score is claimed. The video combines synchronized top and wrist camera frames.
