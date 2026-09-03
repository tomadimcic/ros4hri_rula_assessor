# Copyright 2026 Entanglenet GmbH
# SPDX-License-Identifier: LGPL-3.0-only
"""Pure RULA scoring (McAtamney & Corlett, 1993). No ROS dependencies, unit-testable.

Angles in degrees. Flexion positive, extension negative. Returns worksheet component
scores, Table A/B/C lookups and the grand score with action level.
"""
from dataclasses import dataclass, field
from typing import List

# Table A: [upper_arm 1..6][lower_arm 1..3][wrist 1..4][wrist_twist 1..2]
TABLE_A = [
    [[(1, 2), (2, 2), (2, 3), (3, 3)], [(2, 2), (2, 2), (3, 3), (3, 3)], [(2, 3), (3, 3), (3, 3), (4, 4)]],
    [[(2, 3), (3, 3), (3, 4), (4, 4)], [(3, 3), (3, 3), (3, 4), (4, 4)], [(3, 4), (4, 4), (4, 4), (5, 5)]],
    [[(3, 3), (4, 4), (4, 4), (5, 5)], [(3, 4), (4, 4), (4, 4), (5, 5)], [(4, 4), (4, 4), (4, 5), (5, 5)]],
    [[(4, 4), (4, 4), (4, 5), (5, 5)], [(4, 4), (4, 4), (4, 5), (5, 5)], [(4, 4), (4, 5), (5, 5), (6, 6)]],
    [[(5, 5), (5, 5), (5, 6), (6, 7)], [(5, 6), (6, 6), (6, 7), (7, 7)], [(6, 6), (6, 7), (7, 7), (7, 8)]],
    [[(7, 7), (7, 7), (7, 8), (8, 9)], [(8, 8), (8, 8), (8, 9), (9, 9)], [(9, 9), (9, 9), (9, 9), (9, 9)]],
]
# Table B: [neck 1..6][trunk 1..6][legs 1..2]
TABLE_B = [
    [(1, 3), (2, 3), (3, 4), (5, 5), (6, 6), (7, 7)],
    [(2, 3), (2, 3), (4, 5), (5, 5), (6, 7), (7, 7)],
    [(3, 3), (3, 4), (4, 5), (5, 6), (6, 7), (7, 7)],
    [(5, 5), (5, 6), (6, 7), (7, 7), (7, 7), (8, 8)],
    [(7, 7), (7, 7), (7, 8), (8, 8), (8, 8), (8, 8)],
    [(8, 8), (8, 8), (8, 8), (8, 9), (9, 9), (9, 9)],
]
# Table C: [score_a 1..8+][score_b 1..7+]
TABLE_C = [
    [1, 2, 3, 3, 4, 5, 5],
    [2, 2, 3, 4, 4, 5, 5],
    [3, 3, 3, 4, 4, 5, 6],
    [3, 3, 3, 4, 5, 6, 6],
    [4, 4, 4, 5, 6, 7, 7],
    [4, 4, 5, 6, 6, 7, 7],
    [5, 5, 6, 6, 7, 7, 7],
    [5, 5, 6, 7, 7, 7, 7],
]


@dataclass
class Angles:
    upper_arm_flexion: float = 0.0      # shoulder flexion(+)/extension(-)
    upper_arm_abduction: float = 0.0
    lower_arm_flexion: float = 90.0     # elbow
    wrist_flexion: float = 0.0
    wrist_deviation: float = 0.0        # radial/ulnar
    wrist_twist: float = 0.0            # pronation/supination from mid-range
    neck_flexion: float = 0.0
    neck_twist: float = 0.0
    neck_side_bend: float = 0.0
    trunk_flexion: float = 0.0
    trunk_twist: float = 0.0
    trunk_side_bend: float = 0.0


@dataclass
class Result:
    upper_arm: int
    lower_arm: int
    wrist: int
    wrist_twist: int
    neck: int
    trunk: int
    legs: int
    table_a: int
    table_b: int
    muscle_use: int
    force_load: int
    score_a: int
    score_b: int
    score: int
    action_level: int
    explanation: str
    notes: List[str] = field(default_factory=list)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def score_upper_arm(a: Angles, shoulder_raised=False, arm_supported=False, abduction_threshold=30.0):
    f = a.upper_arm_flexion
    if -20 <= f <= 20:
        s = 1
    elif f < -20 or f <= 45:
        s = 2
    elif f <= 90:
        s = 3
    else:
        s = 4
    if shoulder_raised:
        s += 1
    if abs(a.upper_arm_abduction) > abduction_threshold:
        s += 1
    if arm_supported:
        s -= 1
    return _clamp(s, 1, 6)


def score_lower_arm(a: Angles, across_midline=False):
    s = 1 if 60 <= a.lower_arm_flexion <= 100 else 2
    if across_midline:
        s += 1
    return _clamp(s, 1, 3)


def score_wrist(a: Angles, deviation_threshold=5.0):
    w = abs(a.wrist_flexion)
    s = 1 if w < 2 else (2 if w <= 15 else 3)
    if abs(a.wrist_deviation) > deviation_threshold:
        s += 1
    return _clamp(s, 1, 4)


def score_wrist_twist(a: Angles):
    return 1 if abs(a.wrist_twist) < 45 else 2


def score_neck(a: Angles):
    n = a.neck_flexion
    if n < -2:
        s = 4                      # extension
    elif n <= 10:
        s = 1
    elif n <= 20:
        s = 2
    else:
        s = 3
    if abs(a.neck_twist) > 10:
        s += 1
    if abs(a.neck_side_bend) > 10:
        s += 1
    return _clamp(s, 1, 6)


def score_trunk(a: Angles, sitting_supported=True):
    t = a.trunk_flexion
    if t <= 2 and sitting_supported:
        s = 1
    elif t <= 20:
        s = 2
    elif t <= 60:
        s = 3
    else:
        s = 4
    if abs(a.trunk_twist) > 10:
        s += 1
    if abs(a.trunk_side_bend) > 10:
        s += 1
    return _clamp(s, 1, 6)


def assess(a: Angles, legs=1, muscle_use=0, force_load=0, **flags) -> Result:
    ua = score_upper_arm(a, flags.get("shoulder_raised", False), flags.get("arm_supported", False))
    la = score_lower_arm(a, flags.get("across_midline", False))
    wr = score_wrist(a)
    wt = score_wrist_twist(a)
    nk = score_neck(a)
    tr = score_trunk(a, flags.get("sitting_supported", True))
    lg = _clamp(legs, 1, 2)
    ta = TABLE_A[ua - 1][la - 1][wr - 1][wt - 1]
    tb = TABLE_B[nk - 1][tr - 1][lg - 1]
    sa = ta + muscle_use + force_load
    sb = tb + muscle_use + force_load
    score = TABLE_C[_clamp(sa, 1, 8) - 1][_clamp(sb, 1, 7) - 1]
    level = 1 if score <= 2 else (2 if score <= 4 else (3 if score <= 6 else 4))
    drivers = []
    if ua >= 3:
        drivers.append(f"upper arm {ua} (flexion {a.upper_arm_flexion:.0f} deg, abduction {a.upper_arm_abduction:.0f} deg)")
    if la >= 2:
        drivers.append(f"lower arm {la} (elbow {a.lower_arm_flexion:.0f} deg)")
    if wr >= 3:
        drivers.append(f"wrist {wr} (flexion {a.wrist_flexion:.0f} deg, deviation {a.wrist_deviation:.0f} deg)")
    if nk >= 3:
        drivers.append(f"neck {nk} (flexion {a.neck_flexion:.0f} deg)")
    if tr >= 3:
        drivers.append(f"trunk {tr} (flexion {a.trunk_flexion:.0f} deg)")
    if muscle_use:
        drivers.append("static posture > 1 min")
    if force_load:
        drivers.append(f"force/load {force_load}")
    expl = "; ".join(drivers) if drivers else "posture within acceptable ranges"
    return Result(ua, la, wr, wt, nk, tr, lg, ta, tb, muscle_use, force_load, sa, sb, score, level, expl)
