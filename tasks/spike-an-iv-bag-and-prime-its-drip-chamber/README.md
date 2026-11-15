# Spike an IV bag and prime its drip chamber

This bimanual task models aseptic preparation of an infusion set. The left arm stabilizes a filled saline bag by its
hanger tab. The right removes the bag-port and giving-set spike caps by their tabs, inserts the untouched spike to its
collar, and primes the chamber to a measured 15–27 mm axial liquid length. Full credit requires both arms to release
and return home without either exposed sterile interface touching a gripper or the nonsterile work tray.

## Reproducible setup

- Nonsterile stainless work tray: 34 × 54 × 0.4 cm, covering `x=0.16–0.50 m`, `y=-0.30–0.24 m`.
- Sealed 500 mL PVC saline bag: 13 × 20 × 2.6 cm, 530 g, centred at `(0.32,0.06) m`, with a 28 × 8 × 20 mm upright
  hanger tab and a 10 mm-diameter × 30 mm spike port pointing toward the right arm.
- Bag-port cap: blue 15 × 13 mm pull-off cylinder with a 10 × 10 × 20 mm grip tab; its seal releases at 3.5 N.
- Sterile giving set: starts at `(0.42,-0.13) m` in a fixed 55 × 110 × 14 mm white molded presentation cradle. A
  shallow 21 mm-wide channel supports the lower half of the chamber. Two 6 × 3 × 6 mm front bumps leave a 14 mm
  centre slot, while one 40 × 4 × 6 mm rear stop bears on the closed roller-clamp housing. A 1 N molded snap retainer
  holds the set against the 0.15 N cap-pull reaction and releases under upward arm pickup. The chamber crown remains
  accessible and the set lifts vertically
  out after uncapping. The cradle supports only the still-capped chamber and tubing. The set has a 25 mm spike, 20 mm-diameter × 45 mm clear drip
  chamber, 39 mm graduated axial span, 10 cm tube stub, and closed roller clamp.
- Spike cap: navy 18 × 10 mm pull-off cylinder with a 10 × 10 × 14 mm tab; its seal releases at 0.15 N so it can be
  removed without dragging the lightweight giving set out of its presentation cradle.
- Fixed layout: `object_jitter=0`.

The capped spike and sealed port are the sterile interfaces. After either cap is removed, that interface must not
touch a robot gripper or the work tray. During priming, keep the chamber's longitudinal axis within 12° of horizontal
and its lower region at least 10 mm below the wetted port. Measure liquid length along the chamber axis from the
outlet-end graduation; this removes ambiguity from camera perspective or chamber rotation.

## Scoring

Contamination irreversibly scores 0 before spike lock if an exposed spike or open port comes within 7 mm of a gripper
TCP, or its surface reaches within 0.5 mm of the nonsterile tray. Liquid overfill above 34 mm also irreversibly scores
0. Otherwise:

| End state | Score |
|---|---:|
| Both protective caps displaced at least 6 mm | 0.25 |
| Spike seated 23–25 mm, collar at the port mouth | 0.50 |
| Uncontaminated seated spike; 15–27 mm axial fill; valid chamber orientation; release, rest and both arms home | 1.00 |

For full score the bag remains inside the tray bounds, both TCPs are at least 4 cm from the chamber and port, both
TCPs finish within 5 cm of their initial poses, and the bag and set rest for one continuous second.

The simulator uses rigid bodies and geometric cap/spike measurements. A conservative swept-clearance proxy models
interface contamination; it does not claim to simulate microorganisms or glove sterility. `priming_meter.py` converts
measured finger travel into graduated axial liquid length only while the spike is locked and chamber orientation and
gravity-head conditions are valid.

## Run

```bash
INSTRUCTION="Prepare the IV set aseptically: hold the saline bag by its hanger tab, remove the blue spike-port cap and the navy giving-set spike cap by their tabs without touching either exposed sterile interface, seat the spike to its collar, keep the chamber axis within 12 degrees of horizontal and its lower region below the port, then squeeze and release until the marked axial liquid length is 15-27 mm. Release everything and return both arms to their starting poses."

# Scripted full-score demonstration
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=spike_an_iv_bag_and_prime_its_drip_chamber.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=720 --instruction "$INSTRUCTION" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/iv-bag-demo

# Required Dream-1 attempt
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=spike_an_iv_bag_and_prime_its_drip_chamber.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=720 --instruction "$INSTRUCTION" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/iv-bag-dream
```

Render submission recordings as H.264, 1280 × 720, `yuv420p`, under 50 MB.

## Reset and real-trial safety

Use a fresh sealed bag and fresh sterile giving set for every real trial. Discard either item if the exposed spike or
open port touched a glove, gripper or work surface. Return the bag and capped giving set to their stated coordinates,
fully attach both protective caps, empty the chamber, and close the roller clamp. Simulation reset restores both cap
seals, the contamination latch, spike joint and liquid length. Follow the product instructions and local clinical
aseptic policy; this benchmark is not a substitute for clinical training.
