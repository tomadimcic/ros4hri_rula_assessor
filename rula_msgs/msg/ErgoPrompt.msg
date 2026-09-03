# Ergonomic prompt for the operator display (VR overlay, console, dashboard).
# Derived from RulaScore with dwell-time hysteresis so it does not flicker.
std_msgs/Header header
uint8 LEVEL_INFO=0        # posture acceptable, informational
uint8 LEVEL_ATTENTION=1   # investigate: RULA 3-4
uint8 LEVEL_WARNING=2     # change posture now: RULA >= 5
uint8 level
uint8 score               # current RULA grand score (0 if unknown)
bool valid                # false while tracking is missing
string title              # short, e.g. "Posture OK" / "Attention" / "Change posture"
string text               # what drives the score, e.g. "upper arm raised 62 deg; neck flexed 24 deg"
float32 since_s           # how long the current level has been active
