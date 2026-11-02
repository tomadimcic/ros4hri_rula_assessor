# Copyright 2026 Entanglenet GmbH
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""Body geometry (T-R08): the arm's joint angles reproduce the arm by forward kinematics, within 0.5 cm.

arm_angles turns shoulder, elbow and hand positions into the joint angles of the ROS4HRI human model, in the joint
convention documented in openxr_body (r_y_shoulder: yaw about the vertical, 0 = arm out to the side, 90 = in front;
r_p_shoulder: polar angle from hanging down; r_r_shoulder: roll about the upper arm, 0 = the elbow bends toward the
yaw's forward direction; r_elbow: flexion, 0 = straight). The forward kinematics below follows that convention, so the
round trip checks the geometry of the angles the RULA assessor and the model receive. elbow_estimate (no arm
trackers) must keep both segment lengths and hang the elbow toward its hint.
"""
import math
import random

import pytest

from ros4hri_rula_assessor.openxr_body import arm_angles, elbow_estimate

L_UPPER, L_FORE = 0.30, 0.27
TOLERANCE = 0.005                                  # m


def forward(polar, azimuth, roll, elbow, shoulder=(0.0, 0.0, 0.0)):
    """Elbow and hand positions of the right arm for the model's joint angles (degrees)."""
    p, a, r, e = (math.radians(v) for v in (polar, azimuth, roll, elbow))
    u = (math.sin(p) * math.sin(a), -math.sin(p) * math.cos(a), -math.cos(p))        # upper arm direction
    fwd = (math.cos(a), math.sin(a), 0.0)                                            # roll reference
    d = sum(f * c for f, c in zip(fwd, u))
    ref = tuple(f - d * c for f, c in zip(fwd, u))
    n = math.sqrt(sum(c * c for c in ref))
    ref = tuple(c / n for c in ref)
    cross = (u[1] * ref[2] - u[2] * ref[1], u[2] * ref[0] - u[0] * ref[2], u[0] * ref[1] - u[1] * ref[0])
    bend = tuple(math.cos(r) * x + math.sin(r) * y for x, y in zip(ref, cross))      # forearm's side, about u
    fore = tuple(math.cos(e) * x + math.sin(e) * y for x, y in zip(u, bend))
    elbow_pos = tuple(s + L_UPPER * c for s, c in zip(shoulder, u))
    return elbow_pos, tuple(el + L_FORE * c for el, c in zip(elbow_pos, fore))


@pytest.mark.parametrize("seed", [1, 2, 3, 4])
def test_angles_reproduce_the_arm(seed):
    rng = random.Random(seed)
    worst = 0.0
    for _ in range(50):
        polar, azimuth = rng.uniform(10.0, 170.0), rng.uniform(-60.0, 150.0)
        roll, elbow = rng.uniform(-150.0, 150.0), rng.uniform(5.0, 150.0)
        shoulder = (rng.uniform(-0.2, 0.2), rng.uniform(-0.3, -0.1), rng.uniform(1.0, 1.5))
        el, hand = forward(polar, azimuth, roll, elbow, shoulder)
        ang = arm_angles(shoulder, el, hand)
        el2, hand2 = forward(ang["polar"], ang["azimuth"], ang["roll"], ang["elbow"], shoulder)
        worst = max(worst, math.dist(el, el2), math.dist(hand, hand2))
    assert worst <= TOLERANCE, f"forward kinematics of the angles off by {worst * 1000:.2f} mm"


def test_elbow_estimate_keeps_the_segments():
    rng = random.Random(7)
    shoulder = (0.0, -0.2, 1.3)
    for _ in range(200):
        reach = rng.uniform(0.15, L_UPPER + L_FORE - 0.02)
        theta, phi = rng.uniform(0.2, 2.9), rng.uniform(-1.0, 2.6)
        hand = (shoulder[0] + reach * math.sin(theta) * math.sin(phi),
                shoulder[1] - reach * math.sin(theta) * math.cos(phi),
                shoulder[2] - reach * math.cos(theta))
        el = elbow_estimate(shoulder, hand, L_UPPER, L_FORE)
        assert abs(math.dist(shoulder, el) - L_UPPER) <= TOLERANCE
        assert abs(math.dist(el, hand) - L_FORE) <= TOLERANCE
        mid = tuple((s + h) / 2 for s, h in zip(shoulder, hand))
        hint = (0.0, -0.35, -1.0)
        side = sum((e - m) * c for e, m, c in zip(el, mid, hint))
        assert side >= -1e-9, "the elbow does not hang toward the hint (down and outward)"
