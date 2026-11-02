#!/usr/bin/env python3
# Copyright 2026 Entanglenet GmbH
# SPDX-License-Identifier: LGPL-3.0-only
"""openxr_body: VR tracking (headset, controllers, trackers) -> REP-155 body (ROS4HRI model).

The VR tracking front-end of the RULA module. It consumes the tracked points that
openxr_ros2_bridge already converted to the ROS frame, so it never touches OpenXR data itself,
and turns a *seated* welder's points into joint angles of the ros4hri human_description model,
published exactly like hri_body_detect would:

  in   /openxr_ros2_bridge/tracking/{head,right_hand,left_hand}          REP-103 frame
       /openxr_ros2_bridge/tracking/{right_elbow,chest}                   optional trackers
       /openxr_ros2_bridge/tracking/{right_upper_arm,right_forearm}       optional segment trackers (see below)
       calibrate_joy_topic (Joy, default /openxr/right_hand/inputs)        grip held 2 s = arm calibration
  out  note_topic (std_msgs/String)            "arm calibrated" etc., shown on the headset panel
  out  /humans/bodies/tracked                 hri_msgs/IdsList
       /humans/bodies/<id>/joint_states       sensor_msgs/JointState   ROS4HRI joints + wrist joints
       /humans/bodies/<id>/urdf               std_msgs/String (latched), robot_state_publisher spawned
       TF world -> body_<id>

Angles (deg, in the joint_states as rad):
  r_p_shoulder  upper-arm flexion  = angle of the upper arm from hanging down, in the sagittal plane
  r_y_shoulder  upper-arm abduction = angle from hanging down, in the frontal plane
  r_elbow       elbow flexion       = 180 - angle(shoulder-elbow-hand)          (0 = straight arm)
  p_head/y_head/r_head              head pitch/yaw/roll relative to the trunk
  waist                             trunk pitch from the chest tracker
  r_wrist_flexion/deviation/twist   hand orientation relative to the forearm (extension of the model)
The elbow tracker is optional: without it the elbow is estimated from the hand with a two-link arm.

Segment trackers: a tracker strapped anywhere on the upper arm and one on the forearm give the direction of each
segment from their orientation, once the way each sits on its segment is known. Calibration pose: sit upright,
look straight ahead, let the right arm hang straight down and relaxed with the controller in a neutral grip, and
hold the grip for calibrate_hold_s (or call ~/calibrate_arm). That stores each segment's "down" axis in its
tracker's frame, the body's forward direction (head yaw) and the neutral wrist; it is saved to calibration_file.
The wrist is then the rotation of the controller relative to the forearm tracker, measured from that neutral grip
about the forearm's own axes (long axis = twist, toward the palm = flexion, sideways = deviation), so it holds in
any arm pose; the palm is taken to face the thigh in the calibration pose.
With both trackers fresh, calibrated and where an arm can put them (forearm tracker within arm_tracker_max_m[0] of the
controller, upper-arm tracker within [1], not at the same spot: a tracker without position tracking reports a
placeholder position), the segment directions replace the fixed-shoulder estimate; the trunk is assumed upright and
facing the calibrated forward direction (no chest tracker). Switching between the two is announced on note_topic.
"""
import json
import math
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped
from hri_msgs.msg import IdsList
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster


def q_to_mat(q):
    x, y, z, w = q
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def mat_col(m, j):
    return (m[0][j], m[1][j], m[2][j])


def v_sub(a, b):
    return tuple(a[i] - b[i] for i in range(3))


def v_norm(a):
    return math.sqrt(sum(c * c for c in a))


def v_unit(a):
    n = v_norm(a) or 1.0
    return tuple(c / n for c in a)


def v_dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def v_cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def mat_vec(m, v):
    return tuple(sum(m[i][j] * v[j] for j in range(3)) for i in range(3))


def mat_t_vec(m, v):
    return tuple(sum(m[j][i] * v[j] for j in range(3)) for i in range(3))


def mat_mul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def mat_t(m):
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))


def rot_z(v, yaw):
    c, s_ = math.cos(yaw), math.sin(yaw)
    return (c * v[0] - s_ * v[1], s_ * v[0] + c * v[1], v[2])


def angle_between(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, v_dot(v_unit(a), v_unit(b))))))


def elbow_estimate(shoulder, hand, l_upper, l_fore, hint=(0.0, -0.35, -1.0)):
    """Two-link arm: elbow position for a hand position, elbow hanging down and slightly outward (right arm)."""
    u = v_sub(hand, shoulder)
    d = max(min(v_norm(u), l_upper + l_fore - 1e-3), abs(l_upper - l_fore) + 1e-3)
    u = v_unit(u)
    a = (l_upper ** 2 - l_fore ** 2 + d * d) / (2 * d)
    b = math.sqrt(max(l_upper ** 2 - a * a, 0.0))
    perp = v_sub(hint, tuple(c * v_dot(hint, u) for c in u))
    perp = v_unit(perp) if v_norm(perp) > 1e-6 else (0.0, -1.0, 0.0)
    return tuple(shoulder[i] + u[i] * a + perp[i] * b for i in range(3))


def arm_angles(shoulder, elbow, hand):
    """Return the right-arm joint angles (deg) of the ros4hri human model and the RULA angles.

    Input: shoulder/elbow/hand positions (ROS frame of the body: x forward, y left, z up); the RULA angles are the
    sagittal (flexion) and frontal (abduction) ones.

    Model chain (verified with TF): r_y_shoulder yaw about vertical (0 = arm out to the side, +90 = in
    front), r_p_shoulder polar angle from hanging down, r_r_shoulder roll about the upper arm (at 0 the
    elbow bends toward rotate_z(yaw)(+x)), r_elbow flexion (0 = straight).
    Returns dict(polar, azimuth, roll, elbow, flexion, abduction).
    """
    up_arm = v_sub(elbow, shoulder)
    fore = v_sub(hand, elbow)
    u = v_unit(up_arm)
    polar = math.degrees(math.acos(max(-1.0, min(1.0, -u[2]))))
    azimuth = math.degrees(math.atan2(u[0], -u[1])) if polar > 3.0 else 90.0
    f_perp = v_sub(fore, tuple(c * v_dot(fore, u) for c in u))
    fwd_ref = (math.cos(math.radians(azimuth)), math.sin(math.radians(azimuth)), 0.0)
    r_perp = v_sub(fwd_ref, tuple(c * v_dot(fwd_ref, u) for c in u))
    roll = 0.0
    if v_norm(f_perp) > 1e-4 and v_norm(r_perp) > 1e-4:
        a, b = v_unit(r_perp), v_unit(f_perp)
        roll = math.degrees(math.atan2(v_dot(v_cross(a, b), u), v_dot(a, b)))
    elbow_flex = 180.0 - angle_between(tuple(-c for c in up_arm), fore)
    flexion = math.degrees(math.atan2(up_arm[0], -up_arm[2]))
    abduction = math.degrees(math.asin(max(-1.0, min(1.0, -u[1]))))     # lateral elevation, right arm outward positive
    return dict(polar=polar, azimuth=azimuth, roll=roll, elbow=elbow_flex, flexion=flexion, abduction=abduction)


class OpenXRBody(Node):

    def __init__(self):
        super().__init__("openxr_body")
        self.declare_parameter("body_id", "welder")
        self.declare_parameter("height_m", 1.75)
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("ids_rate_hz", 10.0)
        self.declare_parameter("input_timeout_s", 0.3)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("body_pose", [-2.0, 1.2, 0.45, 0.0])    # pelvis of the seated welder in world
        self.declare_parameter("tracking_prefix", "/openxr_ros2_bridge/tracking")
        # right shoulder, ROS frame (x fwd, y left, z up), seated 1.75 m
        self.declare_parameter("shoulder_offset", [0.0, -0.19, 1.09])
        self.declare_parameter("upper_arm_m", 0.373)                    # human_description proportions
        self.declare_parameter("forearm_m", 0.233)
        self.declare_parameter("seated", True)
        self.declare_parameter("spawn_state_publisher", True)
        # sign conventions of the human_description joints (verified with TF, see README)
        self.declare_parameter("shoulder_pitch_sign", 1.0)
        self.declare_parameter("shoulder_yaw_sign", 1.0)
        self.declare_parameter("shoulder_roll_sign", 1.0)
        # model: +p_head = extension; we publish "looking down" positive
        self.declare_parameter("head_pitch_sign", -1.0)
        self.declare_parameter("waist_sign", 1.0)
        # wrist angles are measured relative to a neutral grip; set by ~/calibrate_wrist or here
        self.declare_parameter("wrist_offset_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("calibrate_joy_topic", "/openxr/right_hand/inputs")   # "" = service only
        self.declare_parameter("calibrate_axis", 1)                                   # Joy axes[1] = grip
        self.declare_parameter("calibrate_hold_s", 2.0)
        self.declare_parameter("calibration_file", "~/.ros/openxr_body_arm_calibration.json")
        self.declare_parameter("note_topic", "/telecraft/operator_note")
        self.declare_parameter("arm_tracker_max_m", [0.45, 0.8])   # forearm / upper-arm tracker to controller
        p = self.get_parameter
        self.bid = str(p("body_id").value)
        self.timeout = float(p("input_timeout_s").value)
        self.shoulder = tuple(float(v) for v in p("shoulder_offset").value)
        self.l_upper = float(p("upper_arm_m").value)
        self.l_fore = float(p("forearm_m").value)
        self.seated = bool(p("seated").value)
        self.sg = {k: float(p(k).value) for k in ("shoulder_pitch_sign", "shoulder_yaw_sign", "shoulder_roll_sign",
                                                  "head_pitch_sign", "waist_sign")}
        self.wrist_offset = [float(v) for v in p("wrist_offset_deg").value]
        self.raw_wrist = (0.0, 0.0, 0.0)
        self.source = ""
        self.last_note = ("", None)
        self.create_service(Trigger, "~/calibrate_wrist", self.srv_calibrate)
        # segment trackers: "down" axis of each segment in its tracker's frame, and the body's forward yaw
        self.calib = None
        self.calib_file = os.path.expanduser(str(p("calibration_file").value))
        self.arm_max = [float(v) for v in p("arm_tracker_max_m").value]
        try:
            with open(self.calib_file) as f:
                c = json.load(f)
            self.calib = {"ua": tuple(c["upper_arm_axis"]), "fa": tuple(c["forearm_axis"]), "yaw": float(c["yaw"]),
                          "rel0": tuple(tuple(r) for r in c["wrist_neutral"]), "pal": tuple(c["palm_axis"])}
            self.get_logger().info(f"arm calibration loaded from {self.calib_file} (recalibrate after re-strapping)")
        except (OSError, KeyError, ValueError):
            pass
        self.create_service(Trigger, "~/calibrate_arm", self.srv_calibrate_arm)
        self.pub_note = self.create_publisher(String, str(p("note_topic").value), 10)
        self.cal_axis = int(p("calibrate_axis").value)
        self.cal_hold = float(p("calibrate_hold_s").value)
        self.grip_since, self.grip_done = None, False
        joy_topic = str(p("calibrate_joy_topic").value)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.rx = {}
        prefix = str(p("tracking_prefix").value).rstrip("/")
        for key, name in (("head", "head"), ("rhand", "right_hand"), ("lhand", "left_hand"),
                          ("relbow", "right_elbow"), ("chest", "chest"),
                          ("rua", "right_upper_arm"), ("rfa", "right_forearm")):
            self.create_subscription(PoseStamped, f"{prefix}/{name}", lambda m, k=key: self.on_pose(k, m), qos)
        if joy_topic:
            self.create_subscription(Joy, joy_topic, self.on_joy, qos)

        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        js_topic = f"/humans/bodies/{self.bid}/joint_states"
        urdf_topic = f"/humans/bodies/{self.bid}/urdf"
        self.pub_ids = self.create_publisher(IdsList, "/humans/bodies/tracked", 10)
        self.pub_js = self.create_publisher(JointState, js_topic, 10)
        self.pub_urdf = self.create_publisher(String, urdf_topic, latched)

        import xacro  # noqa: WPS433
        tpl = get_package_share_directory("human_description") + "/urdf/human-tpl.xacro"
        mappings = {"id": self.bid, "height": str(float(p("height_m").value))}
        self.urdf = xacro.process_file(tpl, mappings=mappings).toxml()
        self.pub_urdf.publish(String(data=self.urdf))
        self.rsp = None
        if bool(p("spawn_state_publisher").value):
            self.rsp = subprocess.Popen(["ros2", "run", "robot_state_publisher", "robot_state_publisher", "--ros-args",
                                         "-r", f"__node:=robot_state_publisher_body_{self.bid}",
                                         "-p", f"robot_description:={self.urdf}",
                                         "-r", f"joint_states:={js_topic}", "-r", f"robot_description:={urdf_topic}"],
                                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        x, y, z, yaw = [float(v) for v in p("body_pose").value]
        self.static_tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = str(p("world_frame").value)
        t.child_frame_id = f"body_{self.bid}"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.static_tf.sendTransform(t)

        self.create_timer(1.0 / float(p("ids_rate_hz").value), self.pub_tracked)
        self.create_timer(1.0 / float(p("rate_hz").value), self.tick)
        self.n = 0
        self.get_logger().info(f"openxr_body '{self.bid}': {prefix}/* -> ROS4HRI body, seated={self.seated}")

    def srv_calibrate(self, rq, rs):
        if self.source == "arm trackers":
            rs.success, rs.message = False, "with the arm trackers the neutral wrist is part of ~/calibrate_arm"
            return rs
        self.wrist_offset = list(self.raw_wrist)
        rs.success, rs.message = True, f"wrist neutral set to {[round(v, 1) for v in self.wrist_offset]} deg"
        self.get_logger().info(rs.message)
        return rs

    # ---- arm calibration (segment trackers) ---------------------------------
    def note(self, text):
        self.pub_note.publish(String(data=text))
        self.get_logger().info(text)
        self.last_note = (text, self.get_clock().now())

    def calibrate_arm(self):
        rua, rfa, head, rhand = (self.fresh(k) for k in ("rua", "rfa", "head", "rhand"))
        missing = [n for n, v in (("upper-arm tracker", rua), ("forearm tracker", rfa), ("headset", head),
                                  ("controller", rhand)) if v is None]
        if missing:
            return False, "arm calibration failed: no " + ", ".join(missing)
        if not self.arm_trackers_ok(rhand[0], rua, rfa):
            return False, "arm calibration failed: the arm trackers are not on the arm (no position tracking?)"
        down = (0.0, 0.0, -1.0)
        ua = mat_t_vec(q_to_mat(rua[1]), down)                      # segment "down" in the tracker's own frame
        fa = mat_t_vec(q_to_mat(rfa[1]), down)
        hf = mat_col(q_to_mat(head[1]), 0)
        yaw = math.atan2(hf[1], hf[0])
        r_fa = q_to_mat(rfa[1])
        rel0 = mat_mul(mat_t(r_fa), q_to_mat(rhand[1]))             # neutral hand orientation in the forearm frame
        left = mat_t_vec(r_fa, (-math.sin(yaw), math.cos(yaw), 0.0))  # body left (palm normal of the right hand)
        pal = v_unit(v_sub(left, tuple(c * v_dot(left, fa) for c in fa)))
        self.calib = {"ua": ua, "fa": fa, "yaw": yaw, "rel0": rel0, "pal": pal}
        try:
            os.makedirs(os.path.dirname(self.calib_file), exist_ok=True)
            with open(self.calib_file, "w") as f:
                json.dump({"upper_arm_axis": ua, "forearm_axis": fa, "yaw": yaw, "wrist_neutral": rel0,
                           "palm_axis": pal}, f, indent=1)
        except OSError as e:
            self.get_logger().warning(f"calibration not saved: {e}")
        return True, f"arm calibrated (forward {math.degrees(yaw):.0f} deg)"

    def arm_trackers_ok(self, hand, rua, rfa):
        """Both segment trackers fresh and where an arm can put them (see module docstring)."""
        if rua is None or rfa is None:
            return False
        return (math.dist(rfa[0], hand) < self.arm_max[0] and math.dist(rua[0], hand) < self.arm_max[1]
                and math.dist(rua[0], rfa[0]) > 0.03)

    def srv_calibrate_arm(self, rq, rs):
        rs.success, rs.message = self.calibrate_arm()
        self.note(rs.message)
        return rs

    def on_joy(self, msg: Joy):
        grip = msg.axes[self.cal_axis] if len(msg.axes) > self.cal_axis else 0.0
        trigger = msg.axes[0] if msg.axes else 0.0
        now = self.get_clock().now().nanoseconds * 1e-9
        if grip >= 0.8 and trigger < 0.2:                            # never while hold-to-run is pressed
            if self.grip_since is None:
                self.grip_since, self.grip_done = now, False
            elif not self.grip_done and now - self.grip_since >= self.cal_hold:
                self.grip_done = True
                self.note(self.calibrate_arm()[1])
        else:
            self.grip_since = None

    def on_pose(self, key, msg: PoseStamped):
        pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)      # already REP-103
        q = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.rx[key] = (pos, q, self.get_clock().now())

    def fresh(self, key):
        v = self.rx.get(key)
        if v is None or (self.get_clock().now() - v[2]).nanoseconds * 1e-9 > self.timeout:
            return None
        return v

    def pub_tracked(self):
        m = IdsList()
        m.header.stamp = self.get_clock().now().to_msg()
        m.ids = [self.bid] if self.fresh("rhand") is not None else []
        self.pub_ids.publish(m)

    # ---- geometry -----------------------------------------------------------
    @staticmethod
    def hand_axes_ros(q_ros):
        """Return the frame axes of a tracked point in ROS coordinates: pointing axis (+X) and up axis (+Z).

        The bridge's basis change maps OpenXR -Z (pointing) to +X and OpenXR +Y (up) to +Z.
        """
        m = q_to_mat(q_ros)
        return mat_col(m, 0), mat_col(m, 2)

    def tick(self):
        rhand = self.fresh("rhand")
        if rhand is None:
            return
        ang, source = self.compute(rhand)
        calibrated_now = (self.last_note[0].startswith("arm calibrated")
                          and (self.get_clock().now() - self.last_note[1]).nanoseconds < 3e9)
        if source != self.source and "arm trackers" in (source, self.source) and not calibrated_now:
            self.note("RULA: arm trackers in use" if source == "arm trackers"
                      else f"RULA: arm trackers not usable, {source}")
        self.source = source
        self.publish_angles(ang, source)

    def compute(self, rhand):
        """Joint angles from the tracked points. Returns (angles dict, source text)."""
        hand, q_hand, _ = rhand
        point, palm_up = self.hand_axes_ros(q_hand)
        rua, rfa = self.fresh("rua"), self.fresh("rfa")
        relbow = self.fresh("relbow")
        yaw = 0.0
        if self.calib is not None and self.arm_trackers_ok(hand, rua, rfa):
            # segment trackers: directions in the body frame (x = calibrated forward, z up), virtual shoulder at 0
            yaw = self.calib["yaw"]
            u = rot_z(mat_vec(q_to_mat(rua[1]), self.calib["ua"]), -yaw)
            f = rot_z(mat_vec(q_to_mat(rfa[1]), self.calib["fa"]), -yaw)
            shoulder = (0.0, 0.0, 0.0)
            elbow = tuple(self.l_upper * c for c in u)
            hand_pt = tuple(elbow[i] + self.l_fore * f[i] for i in range(3))
            source = "arm trackers"
        else:
            shoulder, hand_pt = self.shoulder, hand
            elbow = relbow[0] if relbow is not None else elbow_estimate(self.shoulder, hand, self.l_upper, self.l_fore)
            source = "elbow tracker" if relbow is not None else "elbow estimated"
        ang = arm_angles(shoulder, elbow, hand_pt)
        if source == "arm trackers":
            # wrist: controller relative to the forearm tracker, from the neutral grip, about the forearm's own axes
            c = self.calib
            d = mat_mul(mat_mul(mat_t(q_to_mat(rfa[1])), q_to_mat(q_hand)), mat_t(c["rel0"]))
            fa, pal = v_unit(c["fa"]), c["pal"]
            third = v_cross(fa, pal)
            pt, nm = mat_vec(d, fa), mat_vec(d, pal)
            ang["wrist"] = (math.degrees(math.atan2(v_dot(pt, pal), v_dot(pt, fa))),      # flexion toward the palm +
                            math.degrees(math.atan2(v_dot(pt, third), v_dot(pt, fa))),    # deviation
                            math.degrees(math.atan2(v_dot(nm, third), v_dot(nm, pal))))   # twist about the forearm
            return self.head_trunk(ang, source, yaw)
        fore = v_sub(hand_pt, elbow)                      # elbow -> hand
        # wrist: hand pointing axis vs forearm direction, relative to the neutral grip
        f = v_unit(fore)
        lateral = v_unit(v_cross((0.0, 0.0, 1.0), f)) if abs(f[2]) < 0.99 else (0.0, 1.0, 0.0)
        vertical = v_cross(f, lateral)
        raw = (math.degrees(math.atan2(v_dot(point, vertical), v_dot(point, f))),
               math.degrees(math.atan2(v_dot(point, lateral), v_dot(point, f))),
               math.degrees(math.atan2(v_dot(palm_up, lateral), v_dot(palm_up, vertical))))
        self.raw_wrist = raw
        ang["wrist"] = tuple(raw[i] - self.wrist_offset[i] for i in range(3))
        return self.head_trunk(ang, source, yaw)

    def head_trunk(self, ang, source, yaw):
        """Neck and trunk angles relative to the body's forward direction (yaw)."""
        head = self.fresh("head")
        chest = self.fresh("chest")
        trunk_pitch = 0.0
        if chest is not None:
            cf, cu = self.hand_axes_ros(chest[1])
            cf, cu = rot_z(cf, -yaw), rot_z(cu, -yaw)
            trunk_pitch = math.degrees(math.atan2(cu[0], cu[2]))      # lean forward positive
        n_pitch = n_yaw = n_roll = 0.0
        if head is not None:
            hf, hu = self.hand_axes_ros(head[1])
            hf, hu = rot_z(hf, -yaw), rot_z(hu, -yaw)
            n_pitch = math.degrees(math.atan2(-hf[2], hf[0])) - trunk_pitch     # looking down positive
            n_yaw = math.degrees(math.atan2(hf[1], hf[0]))
            n_roll = math.degrees(math.atan2(hu[1], hu[2]))
        ang.update(trunk=trunk_pitch, neck=(n_pitch, n_yaw, n_roll))
        return ang, source

    def publish_angles(self, ang, source):
        polar, azimuth, roll, elbow_flex = ang["polar"], ang["azimuth"], ang["roll"], ang["elbow"]
        flexion, abduction = ang["flexion"], ang["abduction"]
        w_flex, w_dev, w_twist = ang["wrist"]
        trunk_pitch = ang["trunk"]
        n_pitch, n_yaw, n_roll = ang["neck"]
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        sg = self.sg
        vals = {
            "waist": sg["waist_sign"] * trunk_pitch, "p_head": sg["head_pitch_sign"] * n_pitch,
            "y_head": n_yaw, "r_head": n_roll,
            "r_p_shoulder": sg["shoulder_pitch_sign"] * polar, "r_y_shoulder": sg["shoulder_yaw_sign"] * azimuth,
            "r_r_shoulder": sg["shoulder_roll_sign"] * roll, "r_elbow": elbow_flex,
            # explicit RULA angles, independent of the model chain and its sign conventions
            "r_upper_arm_flexion": flexion, "r_upper_arm_abduction": abduction,
            "neck_flexion": n_pitch, "neck_twist": n_yaw, "neck_side_bend": n_roll, "trunk_flexion": trunk_pitch,
            "l_p_shoulder": 10.0, "l_y_shoulder": 3.0, "l_r_shoulder": 0.0, "l_elbow": 25.0,
            "r_wrist_flexion": w_flex, "r_wrist_deviation": w_dev, "r_wrist_twist": w_twist,
        }
        # seated: hip chain mirrors the shoulder chain (p = out to the side, y = toward the front)
        thigh = 90.0 if self.seated else 0.0
        vals.update({"r_p_hip": thigh, "l_p_hip": thigh, "r_y_hip": -thigh, "l_y_hip": -thigh,
                     "r_knee": 0.0, "l_knee": 0.0, "r_r_hip": 0.0, "l_r_hip": 0.0})
        for name, deg in vals.items():
            js.name.append(f"{name}_{self.bid}")
            js.position.append(math.radians(deg))
        self.pub_js.publish(js)
        self.n += 1
        if self.n % 150 == 0:
            self.get_logger().info(f"upper arm flex {flexion:.0f} abd {abduction:.0f} (model polar {polar:.0f} "
                                   f"az {azimuth:.0f} roll {roll:.0f}) deg, elbow {elbow_flex:.0f} deg, "
                                   f"wrist {w_flex:.0f}/{w_dev:.0f} deg, neck {n_pitch:.0f} deg, "
                                   f"trunk {trunk_pitch:.0f} deg ({source})")

    def destroy_node(self):
        if self.rsp is not None:
            self.rsp.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OpenXRBody()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
