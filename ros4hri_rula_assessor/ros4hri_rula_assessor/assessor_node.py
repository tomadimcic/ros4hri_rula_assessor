#!/usr/bin/env python3
# Copyright 2026 Entanglenet GmbH
# SPDX-License-Identifier: LGPL-3.0-only
"""ros4hri_rula_assessor: REP-155 body tracking -> real-time RULA risk score.

ARISE reusable module B. Hardware-agnostic: consumes joint angles of the ROS4HRI human
kinematic model, whatever produced them (VR trackers, mocap suit, RGB camera).

In:  /humans/bodies/tracked                 hri_msgs/IdsList        (REP-155)
     /humans/bodies/<id>/joint_states       sensor_msgs/JointState  joint angles [rad]
Out: /human/risk_score                      rula_msgs/RulaScore     one per tracked body, rate_hz

Joint names are parameters (joint_* below) so the mapping to the human_description
URDF of a given deployment is configuration, not code.
"""
import math

import rclpy
from hri_msgs.msg import IdsList
from rclpy.node import Node
from rula_msgs.msg import RulaScore
from sensor_msgs.msg import JointState

from ros4hri_rula_assessor.rula import Angles, assess

ROLES = ["upper_arm_flexion", "upper_arm_abduction", "lower_arm_flexion", "wrist_flexion",
         "wrist_deviation", "wrist_twist", "neck_flexion", "neck_twist", "neck_side_bend",
         "trunk_flexion", "trunk_twist", "trunk_side_bend"]


class RulaAssessor(Node):
    def __init__(self):
        super().__init__("ros4hri_rula_assessor")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("side", "right")
        self.declare_parameter("input_timeout_s", 0.5)
        self.declare_parameter("static_posture_s", 60.0)
        self.declare_parameter("force_load", 0)
        self.declare_parameter("legs", 1)
        self.declare_parameter("bodies_topic", "/humans/bodies/tracked")
        side = str(self.get_parameter("side").value)
        pre = "r_" if side == "right" else "l_"
        defaults = {
            "upper_arm_flexion": pre + "shoulder_flexion", "upper_arm_abduction": pre + "shoulder_abduction",
            "lower_arm_flexion": pre + "elbow_flexion", "wrist_flexion": pre + "wrist_flexion",
            "wrist_deviation": pre + "wrist_deviation", "wrist_twist": pre + "wrist_twist",
            "neck_flexion": "neck_flexion", "neck_twist": "neck_twist", "neck_side_bend": "neck_side_bend",
            "trunk_flexion": "trunk_flexion", "trunk_twist": "trunk_twist", "trunk_side_bend": "trunk_side_bend",
        }
        self.joint_map = {}
        for role in ROLES:
            self.declare_parameter("joint_" + role, defaults[role])
            self.joint_map[role] = str(self.get_parameter("joint_" + role).value)
        p = self.get_parameter
        self.side = side
        self.timeout = float(p("input_timeout_s").value)
        self.static_s = float(p("static_posture_s").value)
        self.force = int(p("force_load").value)
        self.legs = int(p("legs").value)

        self.bodies = {}          # id -> dict(sub, js, rx_time, posture_key, posture_since)
        self.create_subscription(IdsList, str(p("bodies_topic").value), self.on_ids, 10)
        self.pub = self.create_publisher(RulaScore, "/human/risk_score", 10)
        self.create_timer(1.0 / float(p("rate_hz").value), self.tick)
        self.get_logger().info(f"RULA assessor: side={side}, {p('rate_hz').value} Hz, "
                               f"joints e.g. {self.joint_map['upper_arm_flexion']}, {self.joint_map['neck_flexion']}")

    def on_ids(self, msg: IdsList):
        for bid in msg.ids:
            if bid not in self.bodies:
                topic = f"/humans/bodies/{bid}/joint_states"
                sub = self.create_subscription(JointState, topic, lambda m, b=bid: self.on_js(b, m), 10)
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

    def angles_from(self, js: JointState):
        pos = dict(zip(js.name, js.position))
        vals = {}
        for role, joint in self.joint_map.items():
            if joint in pos:
                vals[role] = math.degrees(pos[joint])
        missing = [self.joint_map[r] for r in ("upper_arm_flexion", "lower_arm_flexion", "neck_flexion", "trunk_flexion") if r not in vals]
        return Angles(**vals), missing

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
            a, missing = self.angles_from(b["js"])
            if missing:
                m.valid, m.explanation = False, f"missing joints: {', '.join(missing)}"
                self.pub.publish(m)
                continue
            key = None
            r = assess(a, legs=self.legs, muscle_use=0, force_load=self.force)
            key = (r.table_a, r.table_b)
            if b["key"] != key:
                b["key"], b["since"] = key, now
            static = (now - b["since"]).nanoseconds * 1e-9 >= self.static_s
            if static:
                r = assess(a, legs=self.legs, muscle_use=1, force_load=self.force)
            m.header.stamp = b["js"].header.stamp if b["js"].header.stamp.sec else now.to_msg()
            m.valid = True
            m.score, m.action_level = r.score, r.action_level
            m.score_a, m.score_b, m.table_a, m.table_b = r.score_a, r.score_b, r.table_a, r.table_b
            m.upper_arm, m.lower_arm, m.wrist, m.wrist_twist = r.upper_arm, r.lower_arm, r.wrist, r.wrist_twist
            m.neck, m.trunk, m.legs, m.muscle_use, m.force_load = r.neck, r.trunk, r.legs, r.muscle_use, r.force_load
            m.upper_arm_flexion_deg, m.upper_arm_abduction_deg = a.upper_arm_flexion, a.upper_arm_abduction
            m.lower_arm_flexion_deg, m.wrist_flexion_deg, m.wrist_deviation_deg = a.lower_arm_flexion, a.wrist_flexion, a.wrist_deviation
            m.neck_flexion_deg, m.trunk_flexion_deg = a.neck_flexion, a.trunk_flexion
            m.explanation = r.explanation
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
