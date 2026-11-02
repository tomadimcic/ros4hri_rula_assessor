# Copyright 2026 Entanglenet GmbH
# SPDX-License-Identifier: LGPL-3.0-only
"""RULA scoring against the published worksheet (McAtamney & Corlett, 1993), no ROS needed.

The expected values come from the worksheet's ranges and its Tables A, B and C, not from the code. Test cases T-R01
to T-R06 of docs/d22/test_cases.md in the TELEC workspace.
"""
import pytest

from ros4hri_rula_assessor.rula import (Angles, assess, score_lower_arm, score_neck, score_trunk, score_upper_arm,
                                        score_wrist, score_wrist_twist)


def test_neutral_seated_posture_is_acceptable():
    """T-R01: every component in its neutral range gives grand score 1, action level 1."""
    r = assess(Angles(upper_arm_flexion=0, lower_arm_flexion=80, wrist_flexion=0, neck_flexion=5, trunk_flexion=0))
    assert (r.upper_arm, r.lower_arm, r.wrist, r.wrist_twist, r.neck, r.trunk, r.legs) == (1, 1, 1, 1, 1, 1, 1)
    assert (r.table_a, r.table_b, r.score, r.action_level) == (1, 1, 1, 1)
    assert r.explanation == "posture within acceptable ranges"


def test_worked_example_through_tables_a_b_c():
    """T-R02: a high posture scored by hand through the three tables.

    Upper arm 100 deg (4), elbow 110 deg (2), wrist 20 deg (3), twist at the end of its range (2): Table A 5, +1
    static muscle use = score A 6. Neck 25 deg (3), trunk 30 deg (3), legs supported (1): Table B 4, +1 = score B 5.
    Table C (6, 5) = 6: action level 3, the drivers named in the explanation.
    """
    r = assess(Angles(upper_arm_flexion=100, lower_arm_flexion=110, wrist_flexion=20, wrist_twist=60,
                      neck_flexion=25, trunk_flexion=30), legs=1, muscle_use=1)
    assert (r.upper_arm, r.lower_arm, r.wrist, r.wrist_twist, r.neck, r.trunk) == (4, 2, 3, 2, 3, 3)
    assert (r.table_a, r.score_a, r.table_b, r.score_b, r.score, r.action_level) == (5, 6, 4, 5, 6, 3)
    for driver in ("upper arm 4", "lower arm 2", "wrist 3", "neck 3", "trunk 3", "static posture"):
        assert driver in r.explanation


@pytest.mark.parametrize("flexion, score", [(0, 1), (20, 1), (-20, 1), (21, 2), (-21, 2), (45, 2), (46, 3), (90, 3),
                                            (91, 4)])
def test_upper_arm_ranges(flexion, score):
    """T-R03: 20 deg extension to 20 deg flexion 1; more extension or 20-45 deg 2; 45-90 deg 3; over 90 deg 4."""
    assert score_upper_arm(Angles(upper_arm_flexion=flexion)) == score


def test_component_adjustments():
    """T-R03: abducted +1, shoulder raised +1, arm supported -1; across the midline +1; wrist deviated +1."""
    a = Angles(upper_arm_flexion=30, upper_arm_abduction=31)
    assert score_upper_arm(a) == 3
    assert score_upper_arm(a, shoulder_raised=True) == 4
    assert score_upper_arm(a, arm_supported=True) == 2
    assert score_lower_arm(Angles(lower_arm_flexion=80)) == 1
    assert score_lower_arm(Angles(lower_arm_flexion=80), across_midline=True) == 2
    assert score_wrist(Angles(wrist_flexion=10, wrist_deviation=10)) == 3
    assert (score_wrist_twist(Angles(wrist_twist=10)), score_wrist_twist(Angles(wrist_twist=60))) == (1, 2)
    assert score_neck(Angles(neck_flexion=5, neck_twist=20, neck_side_bend=20)) == 3
    assert score_trunk(Angles(trunk_flexion=10, trunk_twist=20, trunk_side_bend=20)) == 4


@pytest.mark.parametrize("elbow, score", [(59, 2), (60, 1), (100, 1), (101, 2)])
def test_lower_arm_ranges(elbow, score):
    """T-R03: elbow flexion 60-100 deg 1, otherwise 2."""
    assert score_lower_arm(Angles(lower_arm_flexion=elbow)) == score


@pytest.mark.parametrize("flexion, score", [(0, 1), (10, 2), (-10, 2), (15, 2), (16, 3), (-16, 3)])
def test_wrist_ranges(flexion, score):
    """T-R03: neutral 1, up to 15 deg flexion or extension 2, more 3."""
    assert score_wrist(Angles(wrist_flexion=flexion)) == score


@pytest.mark.parametrize("flexion, score", [(5, 1), (10, 1), (11, 2), (20, 2), (21, 3), (-10, 4)])
def test_neck_ranges(flexion, score):
    """T-R03: 0-10 deg flexion 1, 10-20 deg 2, over 20 deg 3, extension 4."""
    assert score_neck(Angles(neck_flexion=flexion)) == score


@pytest.mark.parametrize("flexion, score", [(0, 1), (10, 2), (20, 2), (21, 3), (60, 3), (61, 4)])
def test_trunk_ranges(flexion, score):
    """T-R03: upright 1, up to 20 deg flexion 2, 20-60 deg 3, over 60 deg 4."""
    assert score_trunk(Angles(trunk_flexion=flexion)) == score


def test_upright_trunk_scores_one_seated_or_standing():
    """T-R04: the worksheet's upright trunk scores 1 "also if well supported while seated", so standing too."""
    assert score_trunk(Angles(trunk_flexion=0), sitting_supported=True) == 1
    assert score_trunk(Angles(trunk_flexion=0), sitting_supported=False) == 1


def test_extreme_posture_clamps_to_table_c_and_action_level_4():
    """T-R05: scores beyond Table C's edges use its last row and column: grand score 7, action level 4."""
    r = assess(Angles(upper_arm_flexion=120, upper_arm_abduction=60, lower_arm_flexion=130, wrist_flexion=40,
                      wrist_deviation=20, wrist_twist=80, neck_flexion=-20, neck_twist=30, neck_side_bend=30,
                      trunk_flexion=70, trunk_twist=30, trunk_side_bend=30),
               legs=2, muscle_use=1, force_load=3, shoulder_raised=True, across_midline=True)
    assert (r.upper_arm, r.lower_arm, r.wrist, r.neck, r.trunk) == (6, 3, 4, 6, 6)
    assert r.score_a > 8 and r.score_b > 7
    assert (r.score, r.action_level) == (7, 4)


@pytest.mark.parametrize("angles, scores", [
    # upper arm 30 (2), elbow 80 (1), wrist 0 (1): Table A 2 + 1 = 3; neck 5 (1), trunk 0 (1): Table B 1 + 1 = 2;
    # Table C (3, 2) = 3: action level 2
    (dict(upper_arm_flexion=30, lower_arm_flexion=80, neck_flexion=5, trunk_flexion=0), (3, 2, 3, 2)),
    # upper arm 50 (3), elbow 110 (2), wrist 10 (2): Table A 4 + 1 = 5; neck 15 (2), trunk 30 (3): Table B 4 + 1 = 5;
    # Table C (5, 5) = 6: action level 3
    (dict(upper_arm_flexion=50, lower_arm_flexion=110, wrist_flexion=10, neck_flexion=15, trunk_flexion=30),
     (5, 5, 6, 3)),
])
def test_action_levels(angles, scores):
    """T-R06: grand scores worked out by hand through Tables A, B and C; action level 2 for 3-4, 3 for 5-6.

    Action level 1 (grand score 1-2) is T-R01, action level 4 (grand score 7) is T-R05.
    """
    r = assess(Angles(**angles), muscle_use=1)
    assert (r.score_a, r.score_b, r.score, r.action_level) == scores
