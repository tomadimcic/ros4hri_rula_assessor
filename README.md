# ros4hri_rula_assessor

**ARISE reusable module #2 (Tele-Craft, Module B).** Computes the Rapid Upper Limb
Assessment (RULA) in real time from REP-155 (ROS4HRI) body tracking, so an operator is
warned about a harmful posture while they hold it, not after a video review.

Status: package skeleton, M5 thin slice in progress (September 2026).

## Interfaces (planned)

| Direction | Topic | Type | Notes |
|---|---|---|---|
| in | `/humans/bodies/tracked` | `hri_msgs/IdsList` | REP-155 |
| in | `/humans/bodies/<id>/joint_states`, `.../skeleton2d` | `sensor_msgs/JointState`, `hri_msgs/Skeleton2D` | per tracked body |
| out | `/humans/bodies/<id>/rula` | RULA score, Group A/B sub-scores, action level, validity, explanation codes | 10 Hz |
| out | `/human/risk_score` | aggregated risk for the active operator | 10 Hz |

* Joint angles filtered before scoring; ISO 11226 posture thresholds.
* Source-agnostic: VR trackers, motion-capture suits or RGB cameras, as long as they
  publish REP-155 bodies.

## License

LGPL-3.0-only. Developed in the Tele-Craft experiment of the ARISE project
(EU Horizon Europe, arise-middleware.eu) by Entanglenet GmbH and Jazzing Automation.
