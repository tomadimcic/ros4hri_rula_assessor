# ros4hri_rula_assessor

ARISE reusable module B (Tele-Craft experiment). Real-time RULA (Rapid Upper Limb
Assessment, McAtamney & Corlett 1993) from REP-155 body tracking, at 10 Hz.
Hardware-agnostic: it consumes joint angles of the ROS4HRI human model, whatever
produced them (VR trackers, mocap suit, RGB camera).

License: LGPL-3.0-or-later (GNU Lesser General Public License, version 3 or any later version). Two packages in this repository:

* `rula_msgs` — `rula_msgs/RulaScore`: grand score, action level, every worksheet
  component score, the joint angles they came from, validity and a human-readable
  explanation of what drives the score.
* `ros4hri_rula_assessor` — the assessor node, the pure-Python scoring library and a
  synthetic body publisher for tests.

## Nodes

### `assessor`

| | Topic | Type |
|---|---|---|
| in | `/humans/bodies/tracked` | `hri_msgs/IdsList` (REP-155) |
| in | `/humans/bodies/<id>/joint_states` | `sensor_msgs/JointState`, angles in rad |
| out | `/human/risk_score` | `rula_msgs/RulaScore`, one per tracked body |

Joint names are parameters (`joint_upper_arm_flexion`, `joint_neck_flexion`, …) so the
mapping to a deployment's `human_description` URDF is configuration. Defaults:
`r_shoulder_flexion`, `r_shoulder_abduction`, `r_elbow_flexion`, `r_wrist_flexion`,
`r_wrist_deviation`, `r_wrist_twist`, `neck_flexion`, `neck_twist`, `neck_side_bend`,
`trunk_flexion`, `trunk_twist`, `trunk_side_bend`.

Other parameters: `rate_hz` (10), `side` (right), `input_timeout_s` (0.5 → `valid=false`),
`static_posture_s` (60, adds the muscle-use point), `force_load` (0–3), `legs` (1–2).

### `openxr_body`

VR tracking front-end: the tracked points that `openxr_ros2_bridge` already converted to the ROS
frame (`/openxr_ros2_bridge/tracking/{head,right_hand,left_hand,right_elbow,chest}`) → REP-155 body
of a *seated* welder. It never reads OpenXR data itself. Joint angles of the human_description model
(mapping verified against its forward kinematics, < 0.5 cm), plus explicit RULA joints
(`r_upper_arm_flexion`, `r_upper_arm_abduction`, `neck_flexion`, `trunk_flexion`) and wrist
joints from the hand orientation relative to the forearm (`~/calibrate_wrist` sets the neutral
grip). The elbow tracker is optional (two-link estimate without it). `spawn_state_publisher`
adds the skeleton to TF for RViz.

### `prompter`

`RulaScore` → `rula_msgs/ErgoPrompt` on `/human/ergo_prompt`: INFO (RULA ≤ 2), ATTENTION (3–4),
WARNING (≥ 5) with dwell-time hysteresis (`dwell_s` to raise, `clear_s` to lower) and a plain
sentence naming what drives the score. This is the "Guardian Angel" message the VR overlay
renders; the operator console previews it.

### `body_sim`

Synthetic REP-155 body: good posture → overhead reach → moderate → good, 45 s loop,
`dropout_at_s` to exercise the validity flag.

## Scoring

`ros4hri_rula_assessor/rula.py` implements the worksheet: component scores from angles,
Tables A, B and C, muscle-use and force points, action levels 1–4. It has no ROS
dependency and is unit-testable.

## Try it

```bash
ros2 run ros4hri_rula_assessor body_sim
ros2 run ros4hri_rula_assessor assessor
ros2 topic echo /human/risk_score
```

## Tests

`colcon test --packages-select ros4hri_rula_assessor` runs, on every push in CI on ROS 2 Jazzy
(`.github/workflows/ci.yml`, which builds `rula_msgs` and this package):

| Test | What it checks |
| --- | --- |
| `test_flake8`, `test_pep257`, `test_xmllint`, `test_mypy` | ament linters; house style in `setup.cfg` (double quotes, lines up to 120 characters) |
| `test_copyright` | skipped: the sources are LGPL-3.0-only (SPDX headers), which ament_copyright does not recognise |
| `test_rula_worksheet` | the RULA worksheet (McAtamney & Corlett, 1993): every component's ranges and adjustments, Tables A, B and C, action levels, expected values worked out by hand from the published tables |
| `test_openxr_body_arm` | arm calibration and the joint angles of known postures from segment trackers with random strap orientations |
| `test_body_geometry` | the arm's joint angles reproduce shoulder, elbow and hand within 0.5 cm by forward kinematics in the model's joint convention (200 random postures); the elbow estimate without arm trackers keeps both segment lengths |

An upright trunk scores 1 whether the operator sits or stands, as on the worksheet ("also if well
supported while seated"); before 2 Nov 2026 a standing, upright trunk scored 2.
