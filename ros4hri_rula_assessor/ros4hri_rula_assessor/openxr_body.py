#!/usr/bin/env python3
# Copyright 2026 Entanglenet GmbH
# SPDX-License-Identifier: LGPL-3.0-only
"""openxr_body: OpenXR headset / controller / tracker poses -> REP-155 body (ROS4HRI model).

The VR tracking front-end of the RULA module. Works with the real headset client and with
openxr_sim alike. It turns the tracked points of a *seated* welder into joint angles of the
ros4hri human_description model and publishes them exactly like hri_body_detect would:

  in   /openxr/head/pose, /openxr/right_hand/pose, /openxr/left_hand/pose
       /openxr/trackers/right_elbow/pose, /openxr/trackers/chest/pose   (optional)
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
"""
import math
import subprocess

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped
from hri_msgs.msg import IdsList
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster

# OpenXR stage (x right, y up, z back) -> ROS (x fwd, y left, z up)
def xr_to_ros(p):
    return (-p[2], -p[0], p[1])


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
    """Right-arm joint angles (deg) of the ros4hri human model for given shoulder/elbow/hand positions
    (ROS frame of the body: x forward, y left, z up), plus the RULA sagittal/frontal angles.

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
    abduction = math.degrees(math.atan2(-up_arm[1], -up_arm[2]))
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
        self.declare_parameter("shoulder_offset_xr", [0.19, 1.09, 0.0]) # right shoulder in stage frame (seated, 1.75 m)
        self.declare_parameter("upper_arm_m", 0.373)                    # human_description proportions
        self.declare_parameter("forearm_m", 0.233)
        self.declare_parameter("seated", True)
        self.declare_parameter("spawn_state_publisher", True)
        # sign conventions of the human_description joints (verified with TF, see README)
        self.declare_parameter("shoulder_pitch_sign", 1.0)
        self.declare_parameter("shoulder_yaw_sign", 1.0)
        self.declare_parameter("shoulder_roll_sign", 1.0)
        self.declare_parameter("head_pitch_sign", -1.0)   # model: +p_head = extension; we publish "looking down" positive
        self.declare_parameter("waist_sign", 1.0)
        # wrist angles are measured relative to a neutral grip; set by ~/calibrate_wrist or here
        self.declare_parameter("wrist_offset_deg", [0.0, 0.0, 0.0])
        p = self.get_parameter
        self.bid = str(p("body_id").value)
        self.timeout = float(p("input_timeout_s").value)
        self.shoulder = xr_to_ros(tuple(float(v) for v in p("shoulder_offset_xr").value))
        self.l_upper = float(p("upper_arm_m").value)
        self.l_fore = float(p("forearm_m").value)
        self.seated = bool(p("seated").value)
        self.sg = {k: float(p(k).value) for k in ("shoulder_pitch_sign", "shoulder_yaw_sign", "shoulder_roll_sign", "head_pitch_sign", "waist_sign")}
        self.wrist_offset = [float(v) for v in p("wrist_offset_deg").value]
        self.raw_wrist = (0.0, 0.0, 0.0)
        self.create_service(Trigger, "~/calibrate_wrist", self.srv_calibrate)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.rx = {}
        for key, topic in (("head", "/openxr/head/pose"), ("rhand", "/openxr/right_hand/pose"),
                           ("lhand", "/openxr/left_hand/pose"), ("relbow", "/openxr/trackers/right_elbow/pose"),
                           ("chest", "/openxr/trackers/chest/pose")):
            self.create_subscription(PoseStamped, topic, lambda m, k=key: self.on_pose(k, m), qos)

        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        js_topic = f"/humans/bodies/{self.bid}/joint_states"
        urdf_topic = f"/humans/bodies/{self.bid}/urdf"
        self.pub_ids = self.create_publisher(IdsList, "/humans/bodies/tracked", 10)
        self.pub_js = self.create_publisher(JointState, js_topic, 10)
        self.pub_urdf = self.create_publisher(String, urdf_topic, latched)

        import xacro  # noqa: WPS433
        tpl = get_package_share_directory("human_description") + "/urdf/human-tpl.xacro"
        self.urdf = xacro.process_file(tpl, mappings={"id": self.bid, "height": str(float(p("height_m").value))}).toxml()
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
        self.get_logger().info(f"openxr_body '{self.bid}': OpenXR -> ROS4HRI body, seated={self.seated}")

    def srv_calibrate(self, rq, rs):
        self.wrist_offset = list(self.raw_wrist)
        rs.success, rs.message = True, f"wrist neutral set to {[round(v, 1) for v in self.wrist_offset]} deg"
        self.get_logger().info(rs.message)
        return rs

    def on_pose(self, key, msg: PoseStamped):
        pos = xr_to_ros((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
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
    def hand_axes_ros(q_xr):
        """Hand frame axes in ROS coordinates: pointing axis (OpenXR -Z), palm-up axis (OpenXR +Y)."""
        m = q_to_mat(q_xr)
        fwd_xr = tuple(-c for c in mat_col(m, 2))
        up_xr = mat_col(m, 1)
        return xr_to_ros(fwd_xr), xr_to_ros(up_xr)

    def tick(self):
        rhand = self.fresh("rhand")
        if rhand is None:
            return
        hand, q_hand, _ = rhand
        relbow = self.fresh("relbow")
        elbow = relbow[0] if relbow is not None else elbow_estimate(self.shoulder, hand, self.l_upper, self.l_fore)
        ang = arm_angles(self.shoulder, elbow, hand)
        polar, azimuth, roll, elbow_flex = ang["polar"], ang["azimuth"], ang["roll"], ang["elbow"]
        flexion, abduction = ang["flexion"], ang["abduction"]
        up_arm = v_sub(elbow, self.shoulder)              # shoulder -> elbow
        fore = v_sub(hand, elbow)                         # elbow -> hand
        fore = v_sub(hand, elbow)
        # wrist: hand pointing axis vs forearm direction, relative to the neutral grip
        point, palm_up = self.hand_axes_ros(q_hand)
        f = v_unit(fore)
        lateral = v_unit(v_cross((0.0, 0.0, 1.0), f)) if abs(f[2]) < 0.99 else (0.0, 1.0, 0.0)
        vertical = v_cross(f, lateral)
        raw = (math.degrees(math.atan2(v_dot(point, vertical), v_dot(point, f))),
               math.degrees(math.atan2(v_dot(point, lateral), v_dot(point, f))),
               math.degrees(math.atan2(v_dot(palm_up, lateral), v_dot(palm_up, vertical))))
        self.raw_wrist = raw
        w_flex, w_dev, w_twist = (raw[i] - self.wrist_offset[i] for i in range(3))
        # head relative to trunk
        head = self.fresh("head")
        chest = self.fresh("chest")
        trunk_pitch = 0.0
        if chest is not None:
            cf, cu = self.hand_axes_ros(chest[1])
            trunk_pitch = math.degrees(math.atan2(cu[0], cu[2]))      # lean forward positive
        n_pitch = n_yaw = n_roll = 0.0
        if head is not None:
            hf, hu = self.hand_axes_ros(head[1])
            n_pitch = math.degrees(math.atan2(-hf[2], hf[0])) - trunk_pitch     # looking down positive
            n_yaw = math.degrees(math.atan2(hf[1], hf[0]))
            n_roll = math.degrees(math.atan2(hu[1], hu[2]))

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        sg = self.sg
        vals = {
            "waist": sg["waist_sign"] * trunk_pitch, "p_head": sg["head_pitch_sign"] * n_pitch, "y_head": n_yaw, "r_head": n_roll,
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
            self.get_logger().info(f"upper arm flex {flexion:.0f} abd {abduction:.0f} (model polar {polar:.0f} az {azimuth:.0f} roll {roll:.0f}) deg, elbow {elbow_flex:.0f} deg, "
                                   f"wrist {w_flex:.0f}/{w_dev:.0f} deg, neck {n_pitch:.0f} deg, trunk {trunk_pitch:.0f} deg"
                                   + ("" if relbow else " (elbow estimated)"))

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
