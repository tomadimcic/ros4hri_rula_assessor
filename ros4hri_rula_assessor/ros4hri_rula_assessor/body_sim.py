#!/usr/bin/env python3
# Copyright 2026 Entanglenet GmbH
# SPDX-License-Identifier: LGPL-3.0-only
"""body_sim: synthetic REP-155 body for testing the RULA assessor without a headset.

Publishes /humans/bodies/tracked (IdsList) and /humans/bodies/<id>/joint_states with a
deterministic posture timeline: good posture -> overhead reach -> moderate -> good.
Optional tracking dropout (dropout_at_s) to exercise the validity flag.
"""
import math

import rclpy
from hri_msgs.msg import IdsList
from rclpy.node import Node
from sensor_msgs.msg import JointState

# (duration_s, angles in degrees)  keys are the assessor's default right-side joint names
PHASES = [
    (15.0, dict(r_shoulder_flexion=15, r_shoulder_abduction=5, r_elbow_flexion=90, r_wrist_flexion=5,
                r_wrist_deviation=0, r_wrist_twist=10, neck_flexion=8, neck_twist=0, neck_side_bend=0,
                trunk_flexion=3, trunk_twist=0, trunk_side_bend=0)),               # good: RULA ~2
    (15.0, dict(r_shoulder_flexion=65, r_shoulder_abduction=35, r_elbow_flexion=120, r_wrist_flexion=20,
                r_wrist_deviation=10, r_wrist_twist=50, neck_flexion=25, neck_twist=0, neck_side_bend=0,
                trunk_flexion=22, trunk_twist=0, trunk_side_bend=0)),              # reach: RULA 5+
    (15.0, dict(r_shoulder_flexion=35, r_shoulder_abduction=15, r_elbow_flexion=80, r_wrist_flexion=12,
                r_wrist_deviation=0, r_wrist_twist=20, neck_flexion=15, neck_twist=0, neck_side_bend=0,
                trunk_flexion=10, trunk_twist=0, trunk_side_bend=0)),              # moderate: RULA 3-4
]
BLEND_S = 3.0


class BodySim(Node):
    def __init__(self):
        super().__init__("body_sim")
        self.declare_parameter("body_id", "b_sim01")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("ids_rate_hz", 10.0)
        self.declare_parameter("dropout_at_s", -1.0)
        self.declare_parameter("dropout_duration_s", 2.0)
        p = self.get_parameter
        self.bid = str(p("body_id").value)
        self.rate = float(p("rate_hz").value)
        self.dropout_at = float(p("dropout_at_s").value)
        self.dropout_len = float(p("dropout_duration_s").value)
        self.pub_ids = self.create_publisher(IdsList, "/humans/bodies/tracked", 10)
        self.pub_js = self.create_publisher(JointState, f"/humans/bodies/{self.bid}/joint_states", 10)
        self.k = 0
        self.total = sum(d for d, _ in PHASES)
        self.create_timer(1.0 / float(p("ids_rate_hz").value), self.pub_tracked)
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(f"body_sim '{self.bid}': {len(PHASES)} phases, {self.total:.0f} s loop")

    def pub_tracked(self):
        m = IdsList()
        m.header.stamp = self.get_clock().now().to_msg()
        m.ids = [self.bid]
        self.pub_ids.publish(m)

    def posture(self, t):
        t = t % self.total
        acc = 0.0
        for i, (d, ang) in enumerate(PHASES):
            if t < acc + d:
                nxt = PHASES[(i + 1) % len(PHASES)][1]
                into_blend = t - (acc + d - BLEND_S)
                if into_blend > 0:
                    s = 0.5 - 0.5 * math.cos(math.pi * into_blend / BLEND_S)
                    return {k: ang[k] + (nxt[k] - ang[k]) * s for k in ang}, i
                return dict(ang), i
            acc += d
        return dict(PHASES[-1][1]), len(PHASES) - 1

    def tick(self):
        k = self.k
        self.k += 1
        t = k / self.rate
        if self.dropout_at >= 0 and self.dropout_at <= t < self.dropout_at + self.dropout_len:
            return
        ang, phase = self.posture(t)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = f"body_{self.bid}"
        for name, deg in ang.items():
            wobble = 1.5 * math.sin(0.8 * t + hash(name) % 7)
            js.name.append(name)
            js.position.append(math.radians(deg + wobble))
        self.pub_js.publish(js)
        if k % int(self.rate * 5) == 0:
            self.get_logger().info(f"t={t:5.1f}s phase {phase}: shoulder {ang['r_shoulder_flexion']:.0f} deg, "
                                   f"neck {ang['neck_flexion']:.0f} deg, trunk {ang['trunk_flexion']:.0f} deg")


def main(args=None):
    rclpy.init(args=args)
    node = BodySim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
