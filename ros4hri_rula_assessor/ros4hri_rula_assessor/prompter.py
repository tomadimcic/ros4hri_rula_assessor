#!/usr/bin/env python3
# Copyright 2026 Entanglenet GmbH
# SPDX-License-Identifier: LGPL-3.0-only
"""prompter: RulaScore -> ErgoPrompt for the operator's display ("Guardian Angel").

Levels: INFO (score <= attention_score-1), ATTENTION (3-4), WARNING (>= warning_score).
A level must persist for `dwell_s` before it is raised and for `clear_s` before it is lowered,
so the prompt does not flicker with every sample. Published at `rate_hz` on /human/ergo_prompt.
"""
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from rula_msgs.msg import ErgoPrompt, RulaScore


def short_drivers(explanation: str) -> str:
    """'upper arm 4 (flexion 65 deg, abduction 35 deg); neck 3 (flexion 25 deg)' -> readable prompt."""
    parts = []
    for item in explanation.split(";"):
        item = item.strip()
        m = re.match(r"(upper arm|lower arm|wrist|neck|trunk) (\d)(?: \((.*)\))?", item)
        if not m:
            if item and not item.startswith("posture within") and not item.startswith("wrist not tracked"):
                parts.append(item)
            continue
        part, detail = m.group(1), m.group(3) or ""
        angle = re.search(r"flexion (-?\d+) deg", detail)
        deg = f" {angle.group(1)} deg" if angle else ""
        parts.append({"upper arm": f"upper arm raised{deg}", "lower arm": "elbow angle",
                      "wrist": f"wrist bent{deg}", "neck": f"neck flexed{deg}", "trunk": f"trunk bent{deg}"}[part])
    return ", ".join(parts)


class Prompter(Node):
    def __init__(self):
        super().__init__("rula_prompter")
        self.declare_parameter("warning_score", 5)
        self.declare_parameter("attention_score", 3)
        self.declare_parameter("dwell_s", 1.5)
        self.declare_parameter("clear_s", 3.0)
        self.declare_parameter("rate_hz", 5.0)
        p = self.get_parameter
        self.warn = int(p("warning_score").value)
        self.attn = int(p("attention_score").value)
        self.dwell = float(p("dwell_s").value)
        self.clear = float(p("clear_s").value)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(RulaScore, "/human/risk_score", self.on_score, 10)
        self.pub = self.create_publisher(ErgoPrompt, "/human/ergo_prompt", latched)
        self.level = ErgoPrompt.LEVEL_INFO
        self.level_since = self.get_clock().now()
        self.candidate = None
        self.candidate_since = None
        self.last = None
        self.create_timer(1.0 / float(p("rate_hz").value), self.tick)
        self.get_logger().info(f"prompter: warning at RULA >= {self.warn}, attention at >= {self.attn}, dwell {self.dwell}s / clear {self.clear}s")

    def on_score(self, m: RulaScore):
        self.last = m
        now = self.get_clock().now()
        if not m.valid:
            target = ErgoPrompt.LEVEL_INFO
        elif m.score >= self.warn:
            target = ErgoPrompt.LEVEL_WARNING
        elif m.score >= self.attn:
            target = ErgoPrompt.LEVEL_ATTENTION
        else:
            target = ErgoPrompt.LEVEL_INFO
        if target == self.level:
            self.candidate = None
            return
        if target != self.candidate:
            self.candidate, self.candidate_since = target, now
            return
        held = (now - self.candidate_since).nanoseconds * 1e-9
        if (target > self.level and held >= self.dwell) or (target < self.level and held >= self.clear):
            self.get_logger().info(f"prompt {self.level} -> {target} (RULA {m.score})")
            self.level, self.level_since, self.candidate = target, now, None

    def tick(self):
        m = ErgoPrompt()
        m.header.stamp = self.get_clock().now().to_msg()
        m.level = self.level
        m.since_s = float((self.get_clock().now() - self.level_since).nanoseconds * 1e-9)
        if self.last is None:
            m.valid, m.title, m.text = False, "No posture data", "waiting for tracking"
        else:
            m.valid, m.score = self.last.valid, self.last.score
            drivers = short_drivers(self.last.explanation) if self.last.valid else "tracking lost"
            if not self.last.valid:
                m.title, m.text = "Tracking lost", "posture cannot be assessed"
            elif self.level == ErgoPrompt.LEVEL_WARNING:
                m.title, m.text = "Change posture now", drivers or f"RULA {self.last.score}"
            elif self.level == ErgoPrompt.LEVEL_ATTENTION:
                m.title, m.text = "Watch your posture", drivers or f"RULA {self.last.score}"
            else:
                m.title, m.text = "Posture OK", f"RULA {self.last.score}, keep going"
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = Prompter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
