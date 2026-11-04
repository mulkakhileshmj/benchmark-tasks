Extend a training handle and engage its detent

Adjustable assistive equipment must lock at a selected setting before release. This unoccupied training module tests pin withdrawal and measured extension followed by positive pin engagement. Extension alone does not earn full credit. It is a tabletop mechanism and must never support a person.

The scripted demonstration reaches 1.0. No model score is claimed. Use the official RoboCurve YAM launchable. Exact installed dependency versions and the source hash are in validation.json. Every solid and initial pose is in scene_inventory.json. Object dimensions are metres. Masses are kilograms. The code in task.py implements the numerical scoring conditions in task.json.

```bash
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python -e tasks/extend-training-handle-and-engage-detent/runtime
source /data/IsaacLab-Arena/yam.env
inspect-robots run --embodiment yam_articulated --policy yam_scripted \
  -E task=tasks/extend-training-handle-and-engage-detent/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=1280 -E cam_height=720 \
  --seed 0 --epochs 1 --instruction "Retract the green locking pin with the left arm. Extend the blue training aid handle to the white 80 mm mark with the right arm. Return the pin into the second hole. Release both grippers with the handle locked at that length. No person is present and the module must not support a person." \
  --scorer task_score --max-steps 600 --store-frames --log-dir runs/demo
```

Run one epoch per process. The successful L40S trial took about 155 seconds while three scenes ran concurrently. The camera videos are native 1280 by 720 at 20 frames per second. Full success includes two seconds of the released state. Reset to the listed starting layout before each trial.

Both uploaded video fields contain the same scripted demonstration. No model execution or model score is claimed. The video combines synchronized top and wrist camera frames.
