# Remove transport keys before unloading a captive carrier

Commissioning and unloading automated equipment requires removing transport restraints before motion. This task tests a physical prerequisite and retention of the removed parts. Newport motion stage documentation describes a bracket that locks the carriage during transport. This is a small unpowered analogue of that prerequisite.

This candidate measures removal of a physical restraint before carrier motion. The lock pins and overhanging lips block extraction. This differs from discovering a lever combination and from simply stowing parts while leaving an arming pin untouched.

## Setup

Use the calibrated bimanual YAM station. All prototype parts are rigid PLA. Ballast the loose parts internally to the stated masses. Secure the base and channel walls and retaining lips and return trays and recovery pad to the table. The horizontal origin is midway between the arm bases. x points away from the robots. y points left. z points up in the station world frame. The table is at z 0.7475 m. carrier has overall dimensions 80 mm by 130 mm by 55 mm. Its centre is x 0.3 m and y 0 m and z 0.795 m. Yaw is 0 radians. It is loose with mass 0.18 kg. stage base has overall dimensions 80 mm by 130 mm by 20 mm. Its centre is x 0.3 m and y 0 m and z 0.7575 m. Yaw is 0 radians. It is fixed. blue key has overall dimensions 26 mm by 22 mm by 71 mm. Its centre is x 0.3 m and y 0.04 m and z 0.785 m. Yaw is 0 radians. It is loose with mass 0.025 kg. orange key has overall dimensions 26 mm by 22 mm by 71 mm. Its centre is x 0.3 m and y -0.04 m and z 0.785 m. Yaw is 0 radians. It is loose with mass 0.025 kg. left return has overall dimensions 160 mm by 110 mm by 22 mm. Its centre is x 0.19 m and y 0.2 m and z 0.7585 m. Yaw is 0 radians. It is fixed. right return has overall dimensions 160 mm by 110 mm by 22 mm. Its centre is x 0.19 m and y -0.2 m and z 0.7585 m. Yaw is 0 radians. It is fixed. recovery pad has overall dimensions 90 mm by 150 mm by 1 mm. Its centre is x 0.17 m and y 0 m and z 0.748 m. Yaw is 0 radians. It is fixed. channel wall right has overall dimensions 130 mm by 12 mm by 44 mm. Its centre is x 0.3 m and y -0.075 m and z 0.7695 m. Yaw is 0 radians. It is fixed. retaining lip right has overall dimensions 130 mm by 26 mm by 6 mm. Its centre is x 0.3 m and y -0.062 m and z 0.7925 m. Yaw is 0 radians. It is fixed. channel wall left has overall dimensions 130 mm by 12 mm by 44 mm. Its centre is x 0.3 m and y 0.075 m and z 0.7695 m. Yaw is 0 radians. It is fixed. retaining lip left has overall dimensions 130 mm by 26 mm by 6 mm. Its centre is x 0.3 m and y 0.062 m and z 0.7925 m. Yaw is 0 radians. It is fixed. Each key stem is 8 mm square and 53 mm long. Each key handle is 26 mm by 22 mm by 18 mm. Two 14 mm square bores pass through the carrier and into the base. Their centres are at y plus or minus 40 mm. Retaining lips prevent lifting the carrier out. Each return tray has a 152 mm by 102 mm inner opening and a 4 mm floor. scene_inventory.json lists every solid in each compound object in local coordinates.

## Scoring

Each correctly stored key is worth 0.25. Its complete bounding box must lie inside its return tray opening. The tray opening spans x 114 to 266 mm. The left opening spans y 149 to 251 mm. The right opening spans y negative 251 to negative 149 mm. Its bottom must be 3 to 7 mm above the table and its top at most 100 mm above the table. The carrier is worth 0.5 after both keys have cleared the fixture and both have been stored. Its centre must be within 6 mm on every axis of x 170 mm and y 0 mm and z 28.5 mm above the table. Tilt and yaw error must each be below 8 degrees. A half turn about z is equivalent. Its bottom must be within 3 mm of the recovery pad top at 1 mm above the table. All credited parts must move below 10 mm per second and 0.1 radians per second. The score is capped at 0.75 until both measured grippers are open beyond 35 mm of finger travel. Full success must hold for 1 second. The entire episode scores zero if the carrier moves more than 5 mm before both keys have been stored or while a key remains engaged. A key is clear when its lowest point rises above 41 mm above the table. A key is considered engaged when its centre remains within 20 mm of its original horizontal centre and its bottom is at most 41 mm above the table. Dropping any loose part more than 8 mm below the tabletop also scores zero. These checks run at 20 Hz. All other outcomes score the sum of the completed credits.

## Simulation fidelity

A scaled rigid body analogue of removing transport locks before unloading a captive carrier. It is not a replica or certification of a commercial motion stage. Primitive cuboids form real bores and retaining lips. There is no teleportation or grasp attachment. PhysX controls the same two YAM arms and joint targets as the official embodiment. Contact offsets are 1 mm on compound colliders. Materials do not deform or break. Mass is distributed uniformly within each compound shape. The task uses simulator default contact material on compound parts and the upstream material on simple fixed cuboids. Contact forces are not graded. Trajectory constraints are sampled at 20 Hz rather than every physics substep.

## Run a trial

Use the prepared official YAM launchable with Isaac Lab Arena. Install this folder's small extension into that environment. It adds compound collider support and trajectory telemetry. It uses the upstream YAM robot assets and action contract.

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e runtime
python validate.py
mkdir -p runs/demo runs/dream
export YAM_TELEMETRY="$PWD/runs/demo/trajectory.jsonl"
export YAM_GEOMETRY_DIR="$PWD/runs/demo/geometry"
inspect-robots run --embodiment yam_industrial --policy yam_scripted -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --seed 0 --epochs 1 --instruction "Remove both transport keys to their return trays before extracting the motion carrier. Pull the carrier through the front opening onto the white recovery pad. Do not move the carrier more than 5 mm while either key is engaged. Release all parts." --scorer task_score --max-steps 1200 --store-frames --log-dir runs/demo
export YAM_TELEMETRY="$PWD/runs/dream/trajectory.jsonl"
export YAM_GEOMETRY_DIR="$PWD/runs/dream/geometry"
inspect-robots run --embodiment yam_industrial --policy dream -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --seed 0 --epochs 1 --instruction "Remove both transport keys to their return trays before extracting the motion carrier. Pull the carrier through the front opening onto the white recovery pad. Do not move the carrier more than 5 mm while either key is engaged. Release all parts." --scorer task_score --max-steps 1200 --store-frames --log-dir runs/dream
```

The Dream credential must be configured privately as described by the program walkthrough. Never put a credential in this folder.

For a counterfactual trial use the scripted policy with task=locked_pull.py. It attempts extraction while both keys remain engaged. The expected score is zero. The validation report records the measured movement.

Convert each saved log with inspect-robots video. The setup screenshot is the first top camera frame of the submitted run.

## Provenance and limits

The full source revision and dependency commits are recorded in validation.json once final evidence is imported. Every score is read from the saved Inspect Robots log. Runtime includes startup and rendering. It is not an estimate from the control horizon.

The scorer uses a one second success hold. Its history resets at the start of every episode. The checks do not establish a maximum contact force or material strength. Reset the full setup between physical trials. The props are unpowered and all loose parts weigh at most 180 g.

Transport restraint reference: [Newport IDL165 LM user manual](https://www.newport.com/medias/sys_master/npresources/h8a/hbb/9954435104798/EDH0360En1030_IDL165-LM%20User%27s%20Manual/EDH0360En1030-IDL165-LM-User-s-Manual.pdf).

## Recorded trials

Demo score was 1.0. The trial used 650 control steps and took 105.58 seconds of wall time. Seed was 0.

Dream score was 0.0. The trial used 1200 control steps and took 192.88 seconds of wall time. Seed was 0.

The matching task source hash is dc32147fe67b15f936857fc9d2adfc9d78f28112f0384362feb93cc815831ed7. Both policies used the same instruction and starting layout. These measurements cover one fixed layout and one seed.

The locked pull counterfactual scored zero. Its maximum measured carrier movement was 4.71 mm. That trial preceded the final support checks. Replaying its recorded poses through the final scorer also gives zero.
