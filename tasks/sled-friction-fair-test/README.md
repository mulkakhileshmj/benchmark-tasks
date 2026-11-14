# Compare sled base friction on a gravity ramp

Category: Other. Material testing requires a controlled comparison before sorting a specimen by measured friction. Both sleds share the ramp slope and start gate. Flat-runout stopping distances establish the comparison. Neither sled receives an imposed launch impulse.

## Fixed setup and scope

The official bimanual YAM station has parallel grippers. The worktop is 0.7475 m high. Two rigid 50 x 50 x 40 mm specimens weigh 150 g and 250 g. Starting centres have x=0.10 m, y=0.14 and 0.05 m. Estimated base friction coefficients are 0.30 and 0.40. Assignments are fixed and disclosed in code.

The 180 x 220 mm ramp slopes 30 degrees, spanning x=0.17 to 0.35 m. Lane centres are y=0.165 and 0.055 m. A 180 x 230 x 8 mm flat runout follows it, with an end stop at x=0.53 m. The shared gate translates sideways by 240 mm when the physical spring button at (0.14,-0.12) m is pressed. A 110 x 100 x 4 mm red mat is centred at (0.23,-0.01) m. Exact collision geometry, estimated materials, joints and motor settings are in task.py and review_native_common.py.

This rigid Coulomb-friction approximation has no wear, deformation, thermal dependence or validated tribology material model. The selected geometry places stopped specimens within the YAM workspace. The shared gate retracts sideways to clear the specimens and robot.

## Run and score

Install runtime and policy into the same isolated Python environment as the official Isaac Lab Arena / Inspect Robots YAM CLI.

~~~bash
uv pip install --python "$VIRTUAL_ENV/bin/python" --no-deps -e runtime -e policy
export PYTHONPATH="$PWD/runtime:$PWD/policy:$PWD:${PYTHONPATH:-}"
export YAM_TELEMETRY="$PWD/demo/trajectory.jsonl"
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment pr74_yam --policy pr74_measured -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=720 --epochs 1 --seed 0 --max-steps 2400 --instruction "$instruction" --scorer task_score --store-frames --log-dir "$PWD/demo"
~~~

For Dream-1 set DREAM_API_KEY in the environment, change policy to dream and use separate logs and telemetry with the same 2,400-step limit. Reset uses a fresh process, seed 0 and zero object jitter.

The scripted expert reads simulator poses and measured stopping events. This privileged pose feedback is disclosed. It never reads the hidden friction assignment. The demonstration compares both stopping distances, then uses a grasp attempt and a physical contact push to place the chosen specimen. Only robot joint commands move specimens. Dream-1 receives its ordinary camera/joint observations.

Scoring grants 0.25 for one valid stopping measurement and 0.5 after both stopping distances differ by more than 30 mm. Full score requires that comparison and only the higher-friction sled stationary on the mat, with hands clear. Mat centre tolerance is 40 mm in each horizontal axis and speed below 10 mm/s. Robot interference and end-stop contact invalidate affected measurements. Final-state scoring has a one-second success hold. Exact predicates are in task.py.

## Evidence

The demo scored 1.0 in 1,263 steps (63.15 simulated seconds). Dream-1 results are in the completed trial JSON and package validation file. Wall time includes renderer startup, frame compression and network inference. Dream SDK retry messages are retained.

Every selected sensor is native 1280 x 720 RGB. Each video is H.264, 20 fps and at most 50 MB. The recording wrapper stores exact RGB arrays in compressed NumPy containers and checks recovered bytes before advancing. It changes neither physics nor actions. The exporter checks dimensions and frame count against each completed trial.

The package contains all three camera streams for each trial, setup screenshots and trial records. Draft metadata, source_manifest.json and evidence.py predate the selected native runs. Final task.json and validation describe the selected evidence.

## Importance evidence

Robot-executed physical comparison of material sliding behaviour under controlled release conditions. See [IMPORTANCE.md](IMPORTANCE.md) for primary sources, the category rationale and remaining review risks.
