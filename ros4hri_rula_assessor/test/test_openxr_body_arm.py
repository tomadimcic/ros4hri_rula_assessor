# Copyright 2026 Entanglenet GmbH
# SPDX-License-Identifier: LGPL-3.0-only
"""openxr_body with arm segment trackers: calibration and joint angles for known postures.

Builds the orientations of the upper-arm tracker, forearm tracker, controller and headset for a right arm from
known joint rotations, with the trackers strapped at random orientations on their segments, the controller held
at a random orientation in the hand and the operator facing a random direction. calibrate_arm() in the hanging
pose plus compute() must then return the joint angles the posture was built from.

Body frame: x forward, y left, z up. Arm hanging, palm facing the thigh: forearm axis = -z, palm normal = +y,
thumb = +x. Flexion of the shoulder or elbow brings the segment forward (about -y), abduction takes the arm out
to the right (about -x); wrist flexion turns the hand toward the palm (about +x), radial deviation toward the
thumb (about -y), twist is about the forearm axis.
"""
import json
import math
import random

import pytest

from ros4hri_rula_assessor.openxr_body import mat_mul, OpenXRBody, q_to_mat

X, Y, Z = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)


def rot(axis, deg):
    a = math.radians(deg)
    c, s, t = math.cos(a), math.sin(a), 1.0 - math.cos(a)
    x, y, z = axis
    return ((c + x * x * t, x * y * t - z * s, x * z * t + y * s),
            (y * x * t + z * s, c + y * y * t, y * z * t - x * s),
            (z * x * t - y * s, z * y * t + x * s, c + z * z * t))


I3 = rot(Z, 0.0)


def mat_to_q(m):
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0.0:
        s = 2.0 * math.sqrt(tr + 1.0)
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s)
    i = max(range(3), key=lambda k: m[k][k])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + m[i][i] - m[j][j] - m[k][k])
    q = [0.0, 0.0, 0.0, 0.0]
    q[i], q[j], q[k], q[3] = 0.25 * s, (m[j][i] + m[i][j]) / s, (m[k][i] + m[i][k]) / s, (m[k][j] - m[j][k]) / s
    return tuple(q)


def random_rot(rng):
    q = [rng.gauss(0.0, 1.0) for _ in range(4)]
    n = math.sqrt(sum(c * c for c in q))
    return q_to_mat(tuple(c / n for c in q))


def make_body(tmp_path):
    b = OpenXRBody.__new__(OpenXRBody)          # geometry only: no ROS node behind it
    b.rx = {}
    b.fresh = b.rx.get                           # every input counts as fresh
    b.l_upper, b.l_fore = 0.373, 0.233
    b.shoulder = (0.0, -0.19, 1.09)
    b.wrist_offset, b.raw_wrist, b.source = [0.0, 0.0, 0.0], (0.0, 0.0, 0.0), ""
    b.calib, b.calib_file = None, str(tmp_path / "arm.json")
    b.arm_max = [0.45, 0.8]
    return b


POS = {"rua": (0.0, -0.2, 1.0), "rfa": (0.1, -0.2, 0.8), "rhand": (0.2, -0.2, 0.7), "head": (0.0, 0.0, 1.3)}


def feed(b, yaw, mounts, sh=I3, el=I3, wr=I3, head=I3, pos=POS):
    """Return the world orientations (ROS frame) of the devices for a posture, as openxr_body receives them.

    The angles come from the orientations only; the positions just have to be where an arm can put the trackers.
    """
    body = rot(Z, yaw)
    ua = mat_mul(body, sh)
    fa = mat_mul(ua, el)
    hand = mat_mul(fa, wr)
    m_ua, m_fa, m_h = mounts
    for key, r in (("rua", mat_mul(ua, m_ua)), ("rfa", mat_mul(fa, m_fa)), ("rhand", mat_mul(hand, m_h)),
                   ("head", mat_mul(body, head))):
        b.rx[key] = (pos[key], mat_to_q(r), None)


POSTURES = [  # (joint rotations, expected angles in deg)
    (dict(sh=rot(Y, -45), el=rot(Y, -90)), dict(flexion=45, abduction=0, elbow=90, wrist=(0, 0, 0))),
    (dict(sh=rot(X, -60), el=rot(Y, -30)), dict(flexion=0, abduction=60, elbow=30, wrist=(0, 0, 0))),
    (dict(sh=rot(Y, 20)), dict(flexion=-20, abduction=0, elbow=0)),
    (dict(sh=rot(Y, -100), el=rot(Y, -10)), dict(flexion=100, abduction=0, elbow=10)),
    (dict(sh=rot(Y, -45), el=rot(Y, -90), wr=rot(X, 30)), dict(flexion=45, elbow=90, wrist=(30, 0, 0))),
    (dict(sh=rot(X, -60), el=rot(Y, -30), wr=rot(X, -25)), dict(wrist=(-25, 0, 0))),
    (dict(sh=rot(Y, -30), el=rot(Y, -60), wr=rot(Y, -15)), dict(wrist=(0, 15, 0))),
    (dict(sh=rot(Y, -45), el=rot(Y, -90), wr=rot(Z, -50)), dict(elbow=90, wrist=(0, 0, 50))),
    (dict(head=rot(Y, 20)), dict(neck=(20, 0, 0), trunk=0)),
    (dict(head=rot(Z, 15)), dict(neck=(0, 15, 0))),
]


@pytest.mark.parametrize("seed", range(8))
def test_arm_trackers_recover_the_posture(tmp_path, seed):
    rng = random.Random(seed)
    yaw = rng.uniform(-180.0, 180.0)                       # operator facing anywhere in the stage
    mounts = (random_rot(rng), random_rot(rng), random_rot(rng))
    b = make_body(tmp_path)
    feed(b, yaw, mounts, head=rot(Y, 10))                  # calibration pose, looking slightly down
    ok, text = b.calibrate_arm()
    assert ok, text
    got = math.degrees(b.calib["yaw"])
    assert math.isclose(got, yaw, abs_tol=1e-6) or abs(abs(got - yaw) - 360) < 1e-6
    for joints, expected in POSTURES:
        feed(b, yaw, mounts, **joints)
        ang, source = b.compute(b.rx["rhand"])
        assert source == "arm trackers"
        for key, want in expected.items():
            got = ang[key]
            if isinstance(want, tuple):
                assert all(abs(g - w) < 1e-6 for g, w in zip(got, want)), (key, joints, got, want)
            else:
                assert abs(got - want) < 1e-6, (key, joints, got, want)


def test_calibration_file_and_fallback(tmp_path):
    rng = random.Random(42)
    mounts = (random_rot(rng), random_rot(rng), random_rot(rng))
    b = make_body(tmp_path)
    feed(b, 30.0, mounts)
    assert b.calibrate_arm()[0]
    with open(b.calib_file) as f:
        saved = json.load(f)
    assert set(saved) == {"upper_arm_axis", "forearm_axis", "yaw", "wrist_neutral", "palm_axis"}
    # without the arm trackers the old estimate takes over
    del b.rx["rua"]
    _, source = b.compute(b.rx["rhand"])
    assert source == "elbow estimated"


def test_calibration_needs_all_devices(tmp_path):
    b = make_body(tmp_path)
    feed(b, 0.0, (I3, I3, I3))
    del b.rx["rfa"]
    ok, text = b.calibrate_arm()
    assert not ok and "forearm tracker" in text and b.calib is None


def test_placeholder_positions_are_not_used(tmp_path):
    """25 Oct 2026: both arm trackers without position tracking, reported 12 mm apart and 0.6 m from the hand."""
    rng = random.Random(3)
    mounts = (random_rot(rng), random_rot(rng), random_rot(rng))
    b = make_body(tmp_path)
    placeholder = dict(POS, rua=(0.45, -0.3, 1.35), rfa=(0.45, -0.312, 1.35))
    feed(b, 0.0, mounts, pos=placeholder)
    ok, text = b.calibrate_arm()
    assert not ok and "not on the arm" in text
    feed(b, 0.0, mounts)                                   # calibrate with real positions ...
    assert b.calibrate_arm()[0]
    feed(b, 0.0, mounts, sh=rot(Y, -45), pos=placeholder)  # ... then the trackers lose their position
    _, source = b.compute(b.rx["rhand"])
    assert source == "elbow estimated"
