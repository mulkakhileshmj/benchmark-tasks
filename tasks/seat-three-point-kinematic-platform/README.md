# Seat a platform on three kinematic datums

The released plate must locate on its cup and groove and flat. Each support is checked independently using contact clearance. No force measurement or force threshold is used.

## Geometry and coordinates

Use the standard YAM station with table top at z 747.5 mm. x points away from the arm bases and y points toward the left arm. Heights in this section are above the table.

| Datum | Centre x and y | Profile |
|---|---|---|
| Cup | 210 mm and plus 50 mm | 40 by 40 mm floor from height 8 to 10 mm. Four 45 degree prism faces rise from the centre at height 10 mm to the outer edges at height 30 mm. Each has 20 mm run and 20 mm rise and 40 mm span. |
| V groove | 310 mm and plus 50 mm | Axis along x. A 50 by 40 mm floor spans height 8 to 10 mm. Two faces run 50 mm along x and rise at 45 degrees from height 10 mm on the centre line to height 30 mm at y offsets plus and minus 20 mm. |
| Flat | 260 mm and minus 50 mm | 50 by 50 mm. Bottom height 8 mm. Top height 14.1421356 mm. |

The base top is 8 mm high. Each ball has radius 10 mm. Seated ball centres are 24.1421356 mm high. Ball centres are 18 mm below the plate centre so the seated plate centre is 42.1421356 mm high. The target contact on an inclined face is 17.0710678 mm high. The balls do not rest on the cup or groove floor.

Foot offsets from the plate centre are x minus 50 mm and y plus 50 mm for the cup foot then x plus 50 mm and y plus 50 mm for the groove foot then x 0 and y minus 50 mm for the flat foot. This identifies the feet by coordinates and avoids inconsistent front and rear labels. task.json lists all dimensions and materials. scene_inventory.json records the authored solids including every prism vertex.

## Contact test and score

In a physical replica check each ball against its corresponding support with a 0.20 mm feeler blade. The blade must not pass between the ball and its mating support without lifting the plate. Check the intended contact location on a cup or groove face rather than the empty space beneath the ball over the floor. This is a clearance test at a stated gauge resolution. It does not estimate normal force. A ruler and level or a calibrated photo check the plate pose. Video establishes release and stillness and the continuous hold.

The simulator uses three separate native contact filters. A filter passes only when it contains a finite contact separation of at most 0.20 mm. A contact against the cone cannot count for the groove or flat. Robot contacts are monitored independently. The scorer samples the latest contacts and pose at 20 Hz.

Any datum displacement over 2 mm or a plate point more than 8 mm below the table or a nonfinite observed pose latches zero. Otherwise the current end state earns:

* 0.25 when the plate's lowest point is more than 15 mm above the table.
* 0.5 when at least one datum contact exists and tilt is below 10 degrees.
* 0.75 when all three contacts exist and each x and y error is below 3 mm and height error below 3 mm and tilt and absolute yaw below 3 degrees and linear speed below 8 mm per second and angular speed below 0.1 radian per second.
* 0.9 when the previous conditions also have every finger joint open beyond 35 mm from its centre and no robot contact within 1 mm of the plate.
* 1.0 once those full conditions hold continuously for one second. Any interruption restarts the hold.

The highest applicable stage is used. All other outcomes score zero. The final score is reported. The numerical score includes the one second hold. The runner may wait for its own success hold before terminating. Simulation contact gaps are a finite resolution proxy for the physical feeler test. Contacts which begin and end between control samples are not guaranteed to be recorded by the scorer.

## Run and reset

Use the official YAM launchable with Arena release 0.3.1 and YAM revision 0059d02a1a91bc6834c1d0846022891df0c78730. Run from this folder with Python 3.12.

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e runtime -e policy
export PYTHONPATH="$PWD/runtime:$PWD/policy:$PWD:$PYTHONPATH"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 YAM_LIMIT_CPU_THREADS=8
python validate.py
TASK_INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_articulated --policy yam_scripted \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=720 --seed 0 --epochs 1 \
  --instruction "$TASK_INSTRUCTION" --scorer task_score --max-steps 2400 \
  --store-frames --log-dir runs/demo
```

For the model use the same command with `--policy dream_current -P server_url=http://127.0.0.1:8241` and a separate output directory. Keep the 2400 step limit. Start the supplied bridge in a separate environment with dreamlabs 1.0.37 and dreamlabs-client 1.1.3 and numpy using `DREAM_BRIDGE_PORT=8241 JAX_PLATFORMS=cpu python policy/current_api_bridge.py`. Configure its credential privately outside the task folder. No scripted fallback is used for the model. Provider session replacements reset model memory and are disclosed in the selected evidence.

Render a finished trial with `inspect-robots video PATH_TO_LOG.json`. Record the setup and overhead and both wrist cameras at native 1280 by 720 and 20 frames per second. Model and demo evidence are separate. The action limit corresponds to 120 seconds of control time. A successful demo can finish early. Wall time depends on rendering and inference.

Reset the level plate to centre x 100 mm and y 0 and height 28 mm with zero yaw. Restore the fixed base and datums and clear history. Nothing is consumed. Use one epoch per process with seed 0 and zero object jitter.

## Validation

validate.py checks independent contact filters and invalid contact data and release and the numeric hold and the analytic datum tangencies and full path reach. Matching new native model and demo receipts accompany this revision. Historical videos are excluded. Source hashes and selected measured results are recorded in validation.json.

The platform has no pose servo and no task callback teleports it. The scripted demonstration uses fixed Cartesian waypoints for this fixed setup. It does not use simulator object poses to correct its actions. Geometry is rigid and omits compliance and manufacturing variation. Use an inert bench replica and keep fingers clear while lowering it.

The selected demo scored 1.0 in 262 steps. Dream 1 scored 0.0 in 2400 steps. Both use the same submitted source and no evaluation errors. There were 0 provider session recoveries in the selected model trial. Each recovery resets model memory without resetting the scene. Held control scored 0.5 and hovering control scored 0.25. These controls do not isolate every predicate and are not a robustness estimate.
