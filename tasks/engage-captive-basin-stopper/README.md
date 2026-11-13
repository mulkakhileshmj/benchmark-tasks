# Reconnect a captive pop-up basin stopper

A household sink stopper can move when its pivot rod lies underneath the stem. In a captive installation the rod must pass through the stem eye. This dry service fixture exposes the side port so a reviewer can see which installation the robot made and measure the resulting stopper travel.

## Bench and reset

Use the fixed bimanual YAM table at 747.5 mm. Place the centre of the stopper eye at world (295, 15, 842.5) mm. The cap is a 35 mm diameter metal disc. Its collar is 14 mm in diameter. The metal drain seat has a 26 mm clear opening and a 42 mm outside diameter. The cap starts seated. Its 5 by 8 mm clear stem eye spans 2.5 mm along the insertion direction. The 4 mm diameter steel pivot rod turns around an X axis placed 25 mm behind the eye along negative Y. Its internal end is 34 mm from the bearing centre. The external grip sleeve is 10 mm in diameter on a 65 mm arm. Start with the bearing translated 16 mm back along negative Y and the lever level. Reset to these positions before every trial. The centre dimensions and complete solid geometry are in `design.py` and `task.py`.

The sectioned stand, guided ball insertion and weak return spring model an accessible service rig. The model represents rod engagement, cap movement and seating. It omits the basin, retaining nut, tap linkage, water and sealing. The scripted policy uses the right YAM gripper. Both grippers are available to the model.

The fixture dimensions are those of this benchmark. Moen documents both ways of installing the rod, including inserting it through the stopper opening for captive operation. This common service choice grounds the societal relatability category. [Moen pop-up waste assembly instructions](https://assets.moen.com/shared/docs/instruction-sheets/ins1355b.pdf)

Use aluminium for the fixed stand, stainless steel for the cap, eye and rod, and a rigid polymer for the grip sleeve. The two upright supports measure 8 by 30 by 105 mm on an 85 by 95 by 15 mm base. The 10 mm ball travels in the side guide. The eye's axial guide allows 35 mm of vertical travel. The lever travels from 3 degrees below level to 32 degrees above it in the lifting direction. Its torsional return rate is 0.5 N m/rad. The exposed cap, ball carriage and lever masses are 39 g, 8 g and 18 g respectively. Normal simulation gravity acts throughout. The simulated contact surfaces share nominal static friction 0.4 and dynamic friction 0.32.

## Score

The scorer awards 0.25 when the bearing is within 0.75 mm of its insertion stop and the shaft clears both eye edges through the stem thickness. Its tip must extend at least 2 mm beyond the far face. The clearance tolerance is 0.15 mm. The score rises to 0.65 when the retained stopper lifts at least 8 mm with the lever tilted at least 20 degrees and the midpoint between each gripper’s tips more than 60 mm from the cap collar. Returning the stopper to within 0.6 mm of its seat and the lever within 4 degrees of level earns 1 after both grippers clear the collar and lever by 60 mm and cap speed remains below 2 mm/s for one second. Loss of capture cancels the opening credit. The exposed eye, cap and lever can be checked with a side photograph, ruler and video.

## Reproduce

In the official YAM launchable, install the local Inspect Robots extensions once. Each trial directory contains its own log and telemetry.

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e runtime -e policy
bash run.sh pilot
bash run.sh demo
# From a separate terminal with the Dream SDK and credentials installed
python policy/current_api_bridge.py
# From the task directory
bash run.sh dream
inspect-robots video PATH_TO_TRIAL_LOG.json --out rendered
```

Trials use seed 0 and 20 Hz. The maximum duration is 120 simulated seconds. The first 200 scripted steps form the pilot. The script can finish before 2,400 steps.

The cap must remain clear throughout a measured opening stroke, starting from its seat. Touching the collar invalidates that stroke. Once a trial process exits, use its native log path with `python evidence.py export runs/demo/TRIAL/adhoc_ID.json` or the corresponding Dream directory. This exports all three native 1280 by 720 camera views and the initial setup frame. The exporter checks frame counts, timestamps, trial status, task score and pre-run source hashes. Copy the entire trial directory to retain the original frame shards and telemetry. Preserve the private session files locally.

## Independent CPU checks

With MuJoCo, NumPy and the official YAM source available, run `YAM_SOURCE=/path/to/yam-isaac-lab-arena python validate.py`. The check drives the external lever with bounded forces, records physical rod and stopper contact, exercises the task scorer and checks every scripted gripper waypoint interpolation with official YAM inverse kinematics. The separate upward load is a one-newton test of the geometry. The native scorer does not require a force sensor. Distances in `cpu_validation.json` are measured from the seated datum. These controls do not establish successful robot grasp and release in Isaac.

## Recorded native trials

The scripted demo scored 1.0 in 351 control steps. Dream 1 scored 0.0 after all 2400 steps. Both used the same task and scorer at 20 Hz with seed 0 and zero object jitter. All three recorded camera streams are H264 at 1280 by 720 pixels. The demo contains 352 frames per camera and the model run 2401.
