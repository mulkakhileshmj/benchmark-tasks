# Set a luggage pull handle and leave it locked

The work outcome is a pull handle at its marked working length with its spring pin engaged after both hands withdraw. This represents an adjustment during luggage service. Reaching the length while leaving the pin retracted does not leave usable equipment. The case is a compact bench prototype held in a service cradle. The trial does not certify luggage strength or service life.

## Importance and construct validity

The earlier [PR 933](https://github.com/robocurve/benchmark-tasks/pull/933) was rejected because it was an abstract pin and rail trainer with no identified equipment. This scene adds the case and working length reference and asks for the equipment outcome rather than giving the manipulation procedure. Its passive pin and sliding handle are working collision geometry. The case body and wheels identify the application but do not add a new physical capability by themselves. Whether this development is sufficiently different and important remains a review decision.

The pin must be withdrawn before the handle can travel. The scorer also records invalid travel while the pin is engaged. Full score needs the pin back in its engaged position and a stationary handle after release. Native contact views check release from both moving bodies. The scorer samples position and the latest contact result at 20 Hz. It does not guarantee detection of a brief event wholly between control samples. No task callback assigns the handle position or switches a visual flag instead of moving a pin.

The model is a compact rigid prototype with an externally accessible spring pin. It does not represent the button linkage or flexible tubes of every commercial suitcase. Wheels are fixed proxies. There is no fabric or deformation or retention load test. The claim is adjustment and geometric engagement rather than proof that the case can carry a specified load. A real prototype can be scored using a ruler and a marked pin and an overhead video. task.json supplies every numeric threshold. scene_inventory.json supplies exact collision geometry and masses and joint anchors.

## Novelty boundary

The prior footrest task in [PR 934](https://github.com/robocurve/benchmark-tasks/pull/934) shared the same pin withdrawal and repositioning sequence and is not counted as an additional distinct submission here. Packing tasks such as [PR 1065](https://github.com/robocurve/benchmark-tasks/pull/1065) have a different equipment outcome from operating the case handle. The inventory of 1741 public PRs was screened on 12 November 2026. No exact luggage pull handle operation was found in that bounded screen. This does not prove global uniqueness or resolve the reviewer's importance judgment.

## Reproduce

Use Arena release 0.3.1 and YAM revision 0059d02a1a91bc6834c1d0846022891df0c78730 on the official YAM launchable with Python 3.12. Run one trial per process from this folder.

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

For Dream 1 install dreamlabs==1.0.37 and dreamlabs-client==1.1.3 with numpy in a separate environment. Configure the existing credential privately. Start `DREAM_BRIDGE_PORT=8241 JAX_PLATFORMS=cpu python policy/current_api_bridge.py` from that environment. Use the simulator command with `--policy dream_current -P server_url=http://127.0.0.1:8241` and a separate log directory. Retain the 2400 step limit. Render each completed log using `inspect-robots video PATH_TO_LOG.json`.

## Selected evidence

The scripted demo scored 1.0 in 352 steps. Dream 1 scored 0.0 in 2400 steps. Both selected trials completed without evaluation errors and use seed 0 and object jitter 0. The matching task source SHA256 is 3f8dd5cacb2bc96f6e9aa767aa45ce6f30292e104d75135cf1ef64abfea27d77.

The model used two documented SDK session recoveries. Each recovery resets model memory. The simulator paused during those requests without resetting the physical scene or providing scripted fallback. This trial is therefore not an uninterrupted provider session. The session IDs and recovery events are preserved in validation.json. The score remains the final score of that completed 2400 step physical trajectory.

The demo took about 86 wall seconds including startup. The model took about 495 wall seconds including startup and provider waits. Both use 20 control Hz. Native H264 videos are 1280 by 720 at 20 frames per second. The model recordings contain 2401 frames and the demo recordings contain 353 frames including initial frames.

Two additional native failure controls changed only the expert actions. Holding the locking pin back scored 0.25 in 506 steps. Keeping the right gripper on the extended handle scored 0.5 in 556 steps. Neither earned full credit. The latter also failed another full condition so it does not isolate the release check by itself. These controls and one successful expert path establish bounded feasibility rather than robustness across arbitrary layouts.

Reset every object and joint to its authored state and clear history. Nothing is consumed. Use inert bench equipment for physical replication. Keep fingers clear of the spring pin and do not use the prototype to support a person or a suspended load.
