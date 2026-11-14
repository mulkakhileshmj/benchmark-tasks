# Score an ampoule neck and open it with a captive opener

An inert ampoule starts unscored. One YAM arm steadies its body while the other inserts a protective opener, rotates its abrasive collar around the neck, clears the scoring edge, and levers off the tip. The body must remain upright in its holder. The separated tip must remain inside the returned opener after both hands release.

## Impact on jobs and scope

Medication-container preparation includes opening ampoules before drawing a dose. This revision covers the subset that requires filing. The [WHO Guide to Good Prescribing, Annex 3](https://iris.who.int/bitstream/handle/10665/59001/WHO_DAP_94.11.pdf) describes filing around an ampoule neck before protected opening. A [WHO biological-standard opening procedure, page 34](https://cdn.who.int/media/docs/default-source/biologicals/bs-documents-%28ecbs%29/2016-documents/ecbs_2016_bs2292_report_who_is_%284th%29_prolactin_19jul16.pdf) specifies scoring with a diamond or tungsten-carbide file. These support including a physical scoring operation before opening.

Many ampoules are already scored or pop-open: the [WHO injection safety toolkit](https://iris.who.int/bitstream/handle/10665/44298/9789241599252_eng.pdf?sequence=1) prefers pop-open products. We do not claim that nurses file every ampoule, or that this exact task is performed by a measured number of workers. The authored collar combines scoring and tip capture in an original training apparatus. It is not an exact commercial device model or a claim of clinical readiness.

## Physics and limitations

All geometry, masses, passive-joint parameters and anchors are in scene_inventory.json. The body is 12 mm in diameter and 40 mm high, with a 6 mm neck; the cap is 20 mm high and at most 9 mm wide. The 24 mm square, 83 mm high opener has four retaining jaws and four beveled, spring-loaded file shoes. Total tool mass is 22 g. The official bimanual YAM station and 0.7475 m tabletop are used.

A file shoe must have native contact with positive normal force at the neck while moving tangentially relative to the body. The integrator measures motion of the shoe body, avoiding apparent travel from contact-centroid jitter. It distributes that travel across 36 angular sectors. Qualification requires at least 34 sectors with 0.25 mm travel each and at least 15 mm total travel. Dark visual sectors indicate qualified coverage; they have no colliders.

The initial native fixed neck joint has a 0.20 N m break torque and 20 N axial break force. Contact-qualified abrasion lowers its torque parameter to 0.020 N m. PhysX must then break the joint through applied robot motion; the controller and scorer never disable the joint or write object poses. Opening before scoring fails permanently.

This is a phenomenological abrasion and breakaway surrogate. It removes no geometry and does not simulate glass cracks or fragments. The abrasion and break thresholds are uncalibrated prototype parameters. Real glass-cutting performance, liquid retention, sterility, sharps containment certification, and clinical effectiveness are outside scope. A physical replica would require measurement and calibration before predictive use.

## Scoring

The executable rules are in scoring.py, scoring_collar.py, task.py, and contact_monitor.py.

- 0.15: cap covered, neck not yet qualified.
- 0.25: scoring qualified and cap covered.
- 0.50: confirmed native neck break, at least 12 mm separation, cap retained and fully covered.
- 0.75: body upright and returned within 2 mm; opener returned within 3 mm and 5 degrees.
- 0.90: hands open and clear, with every physical body/link passing a one-second native-substep rest check.
- 1.00: those release conditions persist for a further second.

An unscored break, direct gripper contact with the cap or neck, cap escape after fracture, body tilt above 15 degrees, drop, or invalid measurement permanently fails the trial. See task.json for all tolerances. No force sensor target earns a grade.

## Reproduce

Run one epoch in a fresh process because the breakaway neck is consumed. Use seed 0, zero object jitter, and the same task instruction for demo and Dream-1.

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e runtime -e policy
export PYTHONPATH="$PWD/runtime:$PWD/policy:$PWD:${PYTHONPATH:-}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 YAM_LIMIT_CPU_THREADS=4
instruction=$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')
export YAM_TELEMETRY="$PWD/runs/demo/telemetry.jsonl"
inspect-robots run --embodiment ampoule_yam --policy yam_scripted \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=720 --seed 0 --epochs 1 \
  --instruction "$instruction" --scorer task_score --max-steps 2400 \
  --store-frames --log-dir runs/demo
```

For Dream-1, configure DREAM_API_KEY privately, replace the policy with dream, and use a fresh telemetry path and output directory. Its limit is 2400 control steps at 20 Hz. Do not substitute a scripted controller during model failure.

Submitted overview videos retain the full native top-camera view and add synchronized left/right wrist insets of 384 by 216 pixels in its upper corners. These are presentation composites; model observations use the original camera streams. There are no time cuts or synthesized frames. make_overview.py reproduces the layout.

The native camera wrapper passes 1280 by 720 into the sensors themselves. Recordings must retain all captured frames at 20 fps and use H.264/yuv420p MP4. The evidence exporter validates dimensions, frame counts, and file size. GPU evidence uses inspect_lossless_frames.py in place of the inspect-robots entry point: it stores each unmodified RGB array in a ZIP container at the original frame path and verifies exact reconstruction. render_evidence.py reads both this lossless store and ordinary NumPy frames. The wrapper changes storage only.

counterexample_no_break.py seats the opener and holds without filing or levering. It uses the same apparatus and changes only robot commands. Run python validate.py, python validate_abrasion.py, and python validate_observed_rest.py to check score semantics, authored geometry/IK, contact-gated integration, and substep-rest guards. These checks do not replace native trials.

## Evidence

The fresh scripted demo scored 1.0 in 782 steps (746.4 seconds wall time). Dream-1 scored 0.0 in 2400 steps (1024.5 seconds wall time). The official client recorded 50 retry attempts; the simulator was not reset and no scripted fallback was used. Both use native 1280 by 720 sensors at 20 Hz. Each of the three camera videos retains every frame, including the starting frame. measured_results.json records selected trials, native qualification/break timing, source hashes and video checks. The 400-step stationary control scored 0.15, qualified only one sector, and never broke the neck.

![Measured task scores](scores.svg)

## Reset and license

Restore body, cap, opener, and all eight passive links to the inventory poses. Clear abrasion history and recreate the native neck joint before physics starts. One epoch per process is required.

Original task code and geometry use LICENSE.txt. Official YAM, Isaac Lab Arena, Inspect Robots and Dream retain their upstream licenses. No manufacturer artwork, logos, medication, patient, or real sharp fragment is included.
