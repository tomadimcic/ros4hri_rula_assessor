# Real-time RULA (Rapid Upper Limb Assessment, McAtamney & Corlett 1993) result for one tracked body.
std_msgs/Header header
string body_id                 # REP-155 body id (/humans/bodies/<id>)
string side                    # "right" | "left" (assessed arm)
uint8 score                    # final RULA grand score 1..7
uint8 action_level             # 1 acceptable, 2 investigate, 3 change soon, 4 change immediately
uint8 score_a                  # posture score A (arm/wrist) incl. muscle use + force
uint8 score_b                  # posture score B (neck/trunk/legs) incl. muscle use + force
uint8 table_a                  # Table A lookup (before muscle/force)
uint8 table_b                  # Table B lookup (before muscle/force)
uint8 upper_arm                # component scores as in the RULA worksheet
uint8 lower_arm
uint8 wrist
uint8 wrist_twist
uint8 neck
uint8 trunk
uint8 legs
uint8 muscle_use
uint8 force_load
float32 upper_arm_flexion_deg  # measured joint angles the scores were derived from
float32 upper_arm_abduction_deg
float32 lower_arm_flexion_deg
float32 wrist_flexion_deg
float32 wrist_deviation_deg
float32 neck_flexion_deg
float32 trunk_flexion_deg
bool valid                     # false when tracking is missing or stale
string explanation             # which components drive the score, human readable
