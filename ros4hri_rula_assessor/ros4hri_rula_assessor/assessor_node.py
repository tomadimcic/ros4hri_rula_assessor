#!/usr/bin/env python3
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
"""ros4hri_rula_assessor: REP-155 body tracking -> real-time RULA risk score.

ARISE reusable module B. Hardware-agnostic: consumes joint angles of the ROS4HRI human
kinematic model, whatever produced them (VR trackers, mocap suit, RGB camera).

In:  /humans/bodies/tracked                 hri_msgs/IdsList        (REP-155)
     /humans/bodies/<id>/joint_states       sensor_msgs/JointState  joint angles [rad]
Out: /human/risk_score                      rula_msgs/RulaScore     one per tracked body, rate_hz

Joint names are parameters (joint_* below) so the mapping to the human_description
URDF of a given deployment is configuration, not code.
"""
from functools import partial
import math

from hri_msgs.msg import IdsList
import rclpy
from rclpy.node import Node
from rula_msgs.msg import RulaScore
from sensor_msgs.msg import JointState

from ros4hri_rula_assessor.rula import Angles, assess

ROLES = ["upper_arm_flexion", "upper_arm_abduction", "lower_arm_flexion", "wrist_flexion",
         "wrist_deviation", "wrist_twist", "neck_flexion", "neck_twist", "neck_side_bend",
         "trunk_flexion", "trunk_twist", "trunk_side_bend"]
REQUIRED = ("upper_arm_flexion", "lower_arm_flexion", "neck_flexion", "trunk_flexion")

# Joint-name templates for the ROS4HRI human kinematic model (ros4hri/human_description,
# urdf/human-tpl.xacro), as published on /humans/bodies/<id>/joint_states by hri_body_detect
# and hri_fullbody. {side} is r/l, {id} the REP-155 body id. The model has no wrist joints,
# so the wrist roles are empty by default; a VR hand-tracking node may add them.
ROS4HRI_JOINTS = {
    # explicit sagittal angle, else shoulder pitch
    "upper_arm_flexion": "{side}_upper_arm_flexion_{id}|{side}_p_shoulder_{id}",
    # explicit frontal angle, else shoulder yaw
    "upper_arm_abduction": "{side}_upper_arm_abduction_{id}|{side}_y_shoulder_{id}",
    "lower_arm_flexion": "{side}_elbow_{id}",           # 0 = straight arm
    "wrist_flexion": "{side}_wrist_flexion_{id}", "wrist_deviation": "{side}_wrist_deviation_{id}",
    "wrist_twist": "{side}_wrist_twist_{id}",      # not in human_description; published by openxr_body
    "neck_flexion": "neck_flexion_{id}|-p_head_{id}",   # explicit, else head pitch (model: + = extension, hence '-')
    "neck_twist": "neck_twist_{id}|y_head_{id}",        # head yaw
    "neck_side_bend": "neck_side_bend_{id}|r_head_{id}",  # head roll
    "trunk_flexion": "trunk_flexion_{id}|waist_{id}",   # waist pitch (+ = lean forward)
    "trunk_twist": "", "trunk_side_bend": "",
}


class RulaAssessor(Node):

    def __init__(self):
        super().__init__("ros4hri_rula_assessor")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("side", "right")
        self.declare_parameter("input_timeout_s", 0.5)
        self.declare_parameter("static_posture_s", 60.0)
        self.declare_parameter("force_load", 0)
        self.declare_parameter("legs", 1)
        self.declare_parameter("sitting_supported", True)
        self.declare_parameter("bodies_topic", "/humans/bodies/tracked")
        side = str(self.get_parameter("side").value)
        self.side_letter = "r" if side == "right" else "l"
        self.declare_parameter("invert_roles", [""])       # roles whose sign must be flipped for this producer
        self.joint_tpl = {}
        for role in ROLES:
            self.declare_parameter("joint_" + role, ROS4HRI_JOINTS[role])
            self.joint_tpl[role] = str(self.get_parameter("joint_" + role).value)
        self.invert = {r for r in self.get_parameter("invert_roles").value if r}
        p = self.get_parameter
        self.side = side
        self.timeout = float(p("input_timeout_s").value)
        self.static_s = float(p("static_posture_s").value)
        self.force = int(p("force_load").value)
        self.legs = int(p("legs").value)
        self.sitting = bool(p("sitting_supported").value)

        self.bodies = {}          # id -> dict(sub, js, rx_time, posture_key, posture_since)
        self.create_subscription(IdsList, str(p("bodies_topic").value), self.on_ids, 10)
        self.pub = self.create_publisher(RulaScore, "/human/risk_score", 10)
        self.create_timer(1.0 / float(p("rate_hz").value), self.tick)
        self.get_logger().info(f"RULA assessor: side={side}, {p('rate_hz').value} Hz, ROS4HRI joints e.g. "
                               f"{self.joint_tpl['upper_arm_flexion']}, {self.joint_tpl['neck_flexion']}")

    def on_ids(self, msg: IdsList):
        for bid in msg.ids:
            if bid not in self.bodies:
                topic = f"/humans/bodies/{bid}/joint_states"
                sub = self.create_subscription(JointState, topic, partial(self.on_js, bid), 10)
                self.bodies[bid] = dict(sub=sub, js=None, rx=None, key=None, since=None)
                self.get_logger().info(f"tracking body '{bid}' via {topic}")
        for bid in list(self.bodies):
            if bid not in msg.ids:
                self.destroy_subscription(self.bodies[bid]["sub"])
                del self.bodies[bid]
                self.get_logger().info(f"body '{bid}' no longer tracked")

    def on_js(self, bid, msg: JointState):
        b = self.bodies.get(bid)
        if b is not None:
            b["js"], b["rx"] = msg, self.get_clock().now()

    def joint_name(self, role, bid, available=None):
        """Return (name, sign) of a role's joint.

        The template may list alternatives separated by '|' (first one present wins); a leading '-' negates
        the value.
        """
        tpl = self.joint_tpl.get(role, "")
        if not tpl:
            return "", 1.0
        cands = []
        for t in tpl.split("|"):
            sign = -1.0 if t.startswith("-") else 1.0
            cands.append((t.lstrip("-").format(side=self.side_letter, id=bid), sign))
        if available is not None:
            for n, sg in cands:
                if n in available:
                    return n, sg
        return cands[0]

    def angles_from(self, js: JointState, bid: str):
        pos = dict(zip(js.name, js.position))
        vals, missing, untracked = {}, [], []
        for role in ROLES:
            name, sign = self.joint_name(role, bid, pos)
            if not name:
                untracked.append(role)
            elif name in pos:
                v = sign * math.degrees(pos[name])
                vals[role] = -v if role in self.invert else v
            elif role in REQUIRED:
                missing.append(name)
            else:
                untracked.append(role)
        return Angles(**vals), missing, untracked

    def tick(self):
        now = self.get_clock().now()
        for bid, b in self.bodies.items():
            m = RulaScore()
            m.header.stamp = now.to_msg()
            m.body_id, m.side = bid, self.side
            if b["js"] is None or (now - b["rx"]).nanoseconds * 1e-9 > self.timeout:
                m.valid, m.explanation = False, "no fresh joint data"
                self.pub.publish(m)
                continue
            a, missing, untracked = self.angles_from(b["js"], bid)
            if missing:
                m.valid, m.explanation = False, f"missing joints: {', '.join(missing)}"
                self.pub.publish(m)
                continue
            key = None
            r = assess(a, legs=self.legs, muscle_use=0, force_load=self.force, sitting_supported=self.sitting)
            key = (r.table_a, r.table_b)
            if b["key"] != key:
                b["key"], b["since"] = key, now
            static = (now - b["since"]).nanoseconds * 1e-9 >= self.static_s
            if static:
                r = assess(a, legs=self.legs, muscle_use=1, force_load=self.force, sitting_supported=self.sitting)
            m.header.stamp = b["js"].header.stamp if b["js"].header.stamp.sec else now.to_msg()
            m.valid = True
            m.score, m.action_level = r.score, r.action_level
            m.score_a, m.score_b, m.table_a, m.table_b = r.score_a, r.score_b, r.table_a, r.table_b
            m.upper_arm, m.lower_arm, m.wrist, m.wrist_twist = r.upper_arm, r.lower_arm, r.wrist, r.wrist_twist
            m.neck, m.trunk, m.legs, m.muscle_use, m.force_load = r.neck, r.trunk, r.legs, r.muscle_use, r.force_load
            m.upper_arm_flexion_deg, m.upper_arm_abduction_deg = a.upper_arm_flexion, a.upper_arm_abduction
            m.lower_arm_flexion_deg, m.wrist_flexion_deg = a.lower_arm_flexion, a.wrist_flexion
            m.wrist_deviation_deg = a.wrist_deviation
            m.neck_flexion_deg, m.trunk_flexion_deg = a.neck_flexion, a.trunk_flexion
            m.explanation = r.explanation
            if untracked and "wrist_flexion" in untracked:
                m.explanation += "; wrist not tracked by this body model"
            self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = RulaAssessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
