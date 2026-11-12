# Aim a registered inspection camera

The work outcome is to aim a camera mount at a second inspection target while retaining its registered pivot position. A displaced mount requires its translational registration to be measured again. The scored ability is coordinated physical adjustment of two passive coupled controls. This is a bench prototype of a mount adjustment operation. It does not measure inspection accuracy or claim a production deployment.

The camera housing is attached to the moving bar. Its geometric axis must intersect the green target. The carriage must stay within 4 mm of its reset datum throughout the motion. The datum is the yaw pivot. It is not the lens entrance pupil. Rotation changes the camera orientation by design. The camera body is a rigid proxy and there is no rendered image sensor or optical calibration model.

## Physical mechanism and validity

Separate passive input forks contact the bar through native rigid body collisions. No task callback assigns an output pose or computes one from the handle coordinates. Moving one input can change both yaw and translation. Moving both can change yaw while retaining the pivot datum. Direct robot contact with the camera bar or its carriage invalidates the trial at the control samples. The contact monitor polls physics contacts. The scorer consumes the latest contact observation at 20 Hz. This does not guarantee detection of a contact which begins and ends between two control samples.

The score requires the camera axis to intersect the target and the mount to settle with both hands clear. Full score is withheld until those conditions hold continuously for one second. The numeric score includes that hold rather than relying only on runner termination. Scoring details and all thresholds are in task.json. A real replica can be judged from a calibrated overhead recording of the pivot and input markers plus a straight alignment rod along the housing axis. That procedure checks geometric aim rather than optical image quality.

## Relationship to existing tasks

This operation develops the rejected abstract trainer in [PR 963](https://github.com/robocurve/benchmark-tasks/pull/963) into a camera mount application with a target and an axis intersection test. The prior final decision is not being represented as a technical approval. Whether this development meets the importance bar remains a review decision.

[PR 370](https://github.com/robocurve/benchmark-tasks/pull/370) steers a laser through two loose mirrors and requires beam blocking during movement. This scene has no mirrors or live laser. Its essential constraint is retaining a moving mount datum through coupled inputs. [PR 1431](https://github.com/robocurve/benchmark-tasks/pull/1431) relocates an extraction hood foot while retaining the mouth position. It does not rotate an output about a retained pivot through two coupled fork contacts. The public inventory of 1741 PRs was screened on 12 November 2026. These close neighbors were inspected individually. This is a bounded novelty assessment rather than proof that no similar task exists elsewhere.

## Reproduce

Use the official YAM launchable with Arena release 0.3.1 and YAM revision 0059d02a1a91bc6834c1d0846022891df0c78730. Python 3.12 is required. Run one trial per process from this folder.

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e runtime -e policy
export PYTHONPATH="$PWD/runtime:$PWD/policy:$PWD:$PYTHONPATH"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 YAM_LIMIT_CPU_THREADS=8
TASK_INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_articulated --policy yam_scripted \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=720 --seed 0 --epochs 1 \
  --instruction "$TASK_INSTRUCTION" --scorer task_score --max-steps 2400 \
  --store-frames --log-dir runs/demo
```

For the model run install dreamlabs==1.0.37 and dreamlabs-client==1.1.3 with numpy in a separate environment. Configure the Dream credential through its normal private credential mechanism. Start the bridge from that environment.

```bash
DREAM_BRIDGE_PORT=8241 JAX_PLATFORMS=cpu python policy/current_api_bridge.py
```

Use the same simulator command in another shell with `--policy dream_current -P server_url=http://127.0.0.1:8241` and a separate log directory. Keep `--max-steps 2400`. Render each finished JSON log with `inspect-robots video PATH_TO_LOG.json`.

## Measured evidence

Both selected trials use seed 0 and object jitter 0 and task source SHA256 a88bbb6d58ef161d2ab6e92a87a64b66a553b6badaf537639b1f437fbc70a5f2. The scripted demo scored 1.0 in 252 steps. The Dream 1 trial scored 0.0 in 2400 steps. Neither selected trial reported an evaluation error. The model trial used one provider session with no session replacement. There is no scripted fallback. The earlier provider error is excluded.

The measured wall time was about 58 seconds for the demo and 377 seconds for the model including process startup. Control time and wall time are different. Both run at 20 control Hz. The selected native videos are H264 at 1280 by 720 and 20 frames per second. The model videos contain 2401 frames and the demo videos contain 253 frames including their initial frames. validation.json and video_audit.json preserve the run receipts and checksums.

A native counterexample which changes only the right input scored zero in 380 steps. It preserves the production physics and scorer and changes the expert actions. It supports the coordination requirement for this fixture. A single successful scripted trajectory and a single model trial do not establish a population success rate or robustness to other layouts.

scene_inventory.json contains the exact authored solids and joint data. Reset all four prop joints to zero and restore the fixed roots and reference markers. Clear scorer history. Nothing is consumed. Use inert prototype equipment for a physical trial. Do not use a live laser or powered inspection equipment.
