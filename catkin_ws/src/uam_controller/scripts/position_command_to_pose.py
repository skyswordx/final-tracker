#!/usr/bin/env python3
"""Bridge quadrotor PositionCommand messages to PoseStamped for legacy controllers."""

from __future__ import annotations

import math
from typing import Optional

import rospy
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand

try:  # 优先使用 tf 的欧拉-四元数转换工具
    from tf.transformations import quaternion_from_euler
except ImportError:  # pragma: no cover - 兜底实现
    import numpy as _np

    def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
        cz = math.cos(yaw * 0.5)
        sz = math.sin(yaw * 0.5)
        cy = math.cos(pitch * 0.5)
        sy = math.sin(pitch * 0.5)
        cx = math.cos(roll * 0.5)
        sx = math.sin(roll * 0.5)
        qw = cx * cy * cz + sx * sy * sz
        qx = sx * cy * cz - cx * sy * sz
        qy = cx * sy * cz + sx * cy * sz
        qz = cx * cy * sz - sx * sy * cz
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= _np.finfo(float).eps:  # type: ignore[attr-defined]
            return 0.0, 0.0, 0.0, 1.0
        return qx / norm, qy / norm, qz / norm, qw / norm


class PositionCommandToPose:
    """Relay node converting PositionCommand to PoseStamped."""

    def __init__(self) -> None:
        self._input_topic = rospy.get_param("~input_topic", "/uam_controller/cmd")
        self._output_topic = rospy.get_param("~output_topic", "/uam_controller/cmd_pose")
        self._default_frame: Optional[str] = rospy.get_param("~frame_id", None)

        queue_size = rospy.get_param("~queue_size", 10)
        latch = rospy.get_param("~latch", False)

        self._publisher = rospy.Publisher(self._output_topic, PoseStamped, queue_size=queue_size, latch=latch)
        self._subscriber = rospy.Subscriber(self._input_topic, PositionCommand, self._callback, queue_size=queue_size)

        rospy.loginfo("PositionCommand->Pose bridge listening on %s, publishing to %s", self._input_topic, self._output_topic)

    def _callback(self, msg: PositionCommand) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        if self._default_frame:
            pose.header.frame_id = self._default_frame

        pose.pose.position.x = msg.position.x
        pose.pose.position.y = msg.position.y
        pose.pose.position.z = msg.position.z

        yaw = msg.yaw if math.isfinite(msg.yaw) else 0.0
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self._publisher.publish(pose)


def main() -> None:
    rospy.init_node("position_command_to_pose")
    PositionCommandToPose()
    rospy.spin()


if __name__ == "__main__":
    main()
