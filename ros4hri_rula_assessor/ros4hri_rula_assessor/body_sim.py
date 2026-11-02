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
"""body_sim: synthetic REP-155 body built on the ROS4HRI human model.

Behaves like hri_body_detect / hri_fullbody for one body:
  /humans/bodies/tracked               hri_msgs/IdsList
  /humans/bodies/<id>/joint_states     sensor_msgs/JointState  (all joints of human_description)
  /humans/bodies/<id>/urdf             std_msgs/String, latched (rendered from human-tpl.xacro)
  TF                                    world -> body_<id>, plus the skeleton via a spawned
                                        robot_state_publisher (same pattern as hri_body_detect)

Posture timeline: good -> overhead reach -> moderate -> good (45 s loop), optional dropout.
"""
import math
import subprocess

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from hri_msgs.msg import IdsList
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

# (duration_s, angles in degrees) keyed by ROS4HRI joint name without the _<id> suffix.
# Unlisted joints stay at their rest value.
REST = dict(waist=2, r_head=0, y_head=0, p_head=8,
            r_y_shoulder=5, r_p_shoulder=15, r_r_shoulder=0, r_elbow=90,
            l_y_shoulder=3, l_p_shoulder=10, l_r_shoulder=0, l_elbow=30,
            r_y_hip=0, r_p_hip=0, r_r_hip=0, r_knee=0, l_y_hip=0, l_p_hip=0, l_r_hip=0, l_knee=0)
PHASES = [
    (15.0, dict(r_p_shoulder=15, r_y_shoulder=5, r_elbow=90, p_head=8, waist=2)),      # good: RULA ~2
    (15.0, dict(r_p_shoulder=65, r_y_shoulder=35, r_elbow=120, p_head=25, waist=22)),  # reach: RULA 5+
    (15.0, dict(r_p_shoulder=35, r_y_shoulder=15, r_elbow=80, p_head=15, waist=10)),   # moderate: RULA 3-4
]
BLEND_S = 3.0


class BodySim(Node):

    def __init__(self):
        super().__init__("body_sim")
        self.declare_parameter("body_id", "b_sim01")
        self.declare_parameter("height_m", 1.75)
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("ids_rate_hz", 10.0)
        self.declare_parameter("dropout_at_s", -1.0)
        self.declare_parameter("dropout_duration_s", 2.0)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("body_pose", [-2.0, 1.5, 0.0, 0.0])   # x y z yaw of body_<id> in world
        self.declare_parameter("spawn_state_publisher", True)
        p = self.get_parameter
        self.bid = str(p("body_id").value)
        self.rate = float(p("rate_hz").value)
        self.dropout_at = float(p("dropout_at_s").value)
        self.dropout_len = float(p("dropout_duration_s").value)

        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        js_topic = f"/humans/bodies/{self.bid}/joint_states"
        urdf_topic = f"/humans/bodies/{self.bid}/urdf"
        self.pub_ids = self.create_publisher(IdsList, "/humans/bodies/tracked", 10)
        self.pub_js = self.create_publisher(JointState, js_topic, 10)
        self.pub_urdf = self.create_publisher(String, urdf_topic, latched)

        # Body model from ros4hri/human_description, exactly as hri_body_detect builds it.
        import xacro  # noqa: WPS433  (ROS xacro python module)
        tpl = get_package_share_directory("human_description") + "/urdf/human-tpl.xacro"
        mappings = {"id": self.bid, "height": str(float(p("height_m").value))}
        self.urdf = xacro.process_file(tpl, mappings=mappings).toxml()
        self.pub_urdf.publish(String(data=self.urdf))

        self.rsp = None
        if bool(p("spawn_state_publisher").value):
            cmd = ["ros2", "run", "robot_state_publisher", "robot_state_publisher", "--ros-args",
                   "-r", f"__node:=robot_state_publisher_body_{self.bid}",
                   "-p", f"robot_description:={self.urdf}",
                   "-r", f"joint_states:={js_topic}", "-r", f"robot_description:={urdf_topic}"]
            self.rsp = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # Place the operator in the world (office side), like a tracked body's root frame.
        x, y, z, yaw = [float(v) for v in p("body_pose").value]
        self.static_tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = str(p("world_frame").value)
        t.child_frame_id = f"body_{self.bid}"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.static_tf.sendTransform(t)

        self.k = 0
        self.total = sum(d for d, _ in PHASES)
        self.create_timer(1.0 / float(p("ids_rate_hz").value), self.pub_tracked)
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(f"body_sim '{self.bid}': ROS4HRI human model ({len(REST)} joints), "
                               f"{len(PHASES)} phases / {self.total:.0f} s loop, body at ({x}, {y}, {z})")

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
                cur = dict(REST, **ang)
                nxt = dict(REST, **PHASES[(i + 1) % len(PHASES)][1])
                into = t - (acc + d - BLEND_S)
                if into > 0:
                    s = 0.5 - 0.5 * math.cos(math.pi * into / BLEND_S)
                    return {k: cur[k] + (nxt[k] - cur[k]) * s for k in cur}, i
                return cur, i
            acc += d
        return dict(REST, **PHASES[-1][1]), len(PHASES) - 1

    def tick(self):
        k = self.k
        self.k += 1
        t = k / self.rate
        if self.dropout_at >= 0 and self.dropout_at <= t < self.dropout_at + self.dropout_len:
            return
        ang, phase = self.posture(t)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        for name, deg in ang.items():
            wobble = 1.5 * math.sin(0.8 * t + (hash(name) % 7))
            js.name.append(f"{name}_{self.bid}")
            js.position.append(math.radians(deg + wobble))
        self.pub_js.publish(js)
        if k % int(self.rate * 5) == 0:
            self.get_logger().info(f"t={t:5.1f}s phase {phase}: r_p_shoulder {ang['r_p_shoulder']:.0f} deg, "
                                   f"p_head {ang['p_head']:.0f} deg, waist {ang['waist']:.0f} deg")

    def destroy_node(self):
        if self.rsp is not None:
            self.rsp.terminate()
        super().destroy_node()


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
