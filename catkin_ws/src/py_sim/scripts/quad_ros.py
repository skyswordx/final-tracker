#!/usr/bin/env python3
"""MuJoCo quadrotor simulator ROS bridge."""

from __future__ import annotations

import os
import time
from typing import Optional

import mujoco
try:
    import mujoco.viewer as mj_viewer
except ImportError:  # 运行在无图形环境时0
    mj_viewer = None

import numpy as np
import rospy
import rospkg
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

try:
    from quadrotor_msgs.msg import SO3Command
except ImportError:  # 仅在未生成消息时提示
    SO3Command = None  # type: ignore

_GRAVITY = 9.8066
_DEFAULT_FORCE_SCALE = 12.5
_MAX_ROTOR_SPEED = 1434.7


class QuadSimBridge:
    """Bridge MuJoCo quadrotor model with ROS topics."""

    def __init__(self) -> None:
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("py_sim")

        self._odom_topic = rospy.get_param("~odom_topic", "/diy_odom")
        self._motor_topic = rospy.get_param("~motor_topic", "/quad/motor_cmd")
        self._command_type = rospy.get_param("~command_type", "motor").lower()
        self._so3_topic = rospy.get_param("~so3_topic", "")

        # 相机话题暂不发布，确保话题命名统一即可
        self._rgb_topic = rospy.get_param("~rgb_topic", "/diy_img/rgb/image_raw")
        self._depth_topic = rospy.get_param("~depth_topic", "/diy_img/depth/image_raw")
        self._camera_info_topic = rospy.get_param("~camera_info_topic", "/diy_img/camera_info")

        default_scene = os.path.join(pkg_path, "scene", "scene_quad.xml")
        self._scene_file = rospy.get_param("~scene_file", default_scene)
        self._physics_dt = rospy.get_param("~physics_dt", 0.005)
        self._enable_viewer = rospy.get_param("~enable_viewer", True)
        self._motor_force_scale = rospy.get_param("~motor_force_scale", _DEFAULT_FORCE_SCALE)
        self._max_rot_speed = rospy.get_param("~max_rot_velocity", _MAX_ROTOR_SPEED)

        if not os.path.isfile(self._scene_file):
            rospy.logfatal("Scene file '%s' does not exist", self._scene_file)
            raise FileNotFoundError(self._scene_file)

        self._model = mujoco.MjModel.from_xml_path(self._scene_file)
        self._data = mujoco.MjData(self._model)

        self._odom_msg = Odometry()
        self._odom_pub = rospy.Publisher(self._odom_topic, Odometry, queue_size=10)

        self._current_motor_cmd = np.zeros(4, dtype=float)
        self._motor_sub = rospy.Subscriber(
            self._motor_topic, Float64MultiArray, self._motor_cb, queue_size=1
        )

        self._so3_sub = None  # type: Optional[rospy.Subscriber]
        if self._command_type == "so3":
            if SO3Command is None:
                rospy.logwarn("SO3Command message not available, falling back to motor commands.")
                self._command_type = "motor"
            elif not self._so3_topic:
                rospy.logwarn("command_type set to 'so3' but ~so3_topic not provided, falling back to motor mode.")
                self._command_type = "motor"
            else:
                self._so3_sub = rospy.Subscriber(
                    self._so3_topic, SO3Command, self._so3_cb, queue_size=1
                )
                rospy.logwarn(
                    "SO3 command support is stubbed; incoming commands will be ignored until mixing is implemented."
                )

        mujoco.set_mjcb_control(self._control_callback)
        self._log_every = rospy.get_param("~log_every", 200)
        self._log_counter = 0

    # ------------------------------------------------------------------
    # ROS Callbacks
    # ------------------------------------------------------------------
    def _motor_cb(self, msg: Float64MultiArray) -> None:
        if not msg.data:
            return
        arr = np.array(msg.data[:4], dtype=float)
        if arr.size < 4:
            rospy.logwarn_once("Received motor command with fewer than 4 entries; ignoring.")
            return
        np.clip(arr, 0.0, 1.0, out=arr)
        self._current_motor_cmd = arr

    def _so3_cb(self, _: object) -> None:
        """Placeholder for SO3 → rotor mapping."""
        rospy.logwarn_throttle(5.0, "SO3 command handling not implemented; ignoring incoming messages.")

    # ------------------------------------------------------------------
    # MuJoCo hooks
    # ------------------------------------------------------------------
    def _control_callback(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:  # noqa: D401
        del model  # model 当前未使用
        self._publish_odometry(data)
        self._apply_motor_commands(data)
        self._log_counter += 1
        if self._log_every > 0 and self._log_counter >= self._log_every:
            self._log_debug()
            self._log_counter = 0

    def _apply_motor_commands(self, data: mujoco.MjData) -> None:
        for i in range(4):
            data.actuator(f"motor{i+1}").ctrl[0] = float(self._current_motor_cmd[i])

        thrusts = np.maximum(self._current_motor_cmd, 0.0) * self._motor_force_scale
        motor_omegas = np.sqrt(np.clip(thrusts, 0.0, None) / max(np.finfo(float).eps, 6.450e-6))
        motor_omegas = np.clip(motor_omegas, 0.0, self._max_rot_speed)

        data.actuator("fl_motor").ctrl[0] = motor_omegas[0]
        data.actuator("fr_motor").ctrl[0] = motor_omegas[1]
        data.actuator("bl_motor").ctrl[0] = motor_omegas[2]
        data.actuator("br_motor").ctrl[0] = motor_omegas[3]

    def _publish_odometry(self, data: mujoco.MjData) -> None:
        msg = self._odom_msg
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"

        msg.pose.pose.position.x = data.qpos[0]
        msg.pose.pose.position.y = data.qpos[1]
        msg.pose.pose.position.z = data.qpos[2]

        msg.pose.pose.orientation.x = data.sensordata[7]
        msg.pose.pose.orientation.y = data.sensordata[8]
        msg.pose.pose.orientation.z = data.sensordata[9]
        msg.pose.pose.orientation.w = data.sensordata[6]

        msg.twist.twist.linear = Vector3(data.qvel[0], data.qvel[1], data.qvel[2])
        msg.twist.twist.angular = Vector3(data.qvel[3], data.qvel[4], data.qvel[5])
        self._odom_pub.publish(msg)

    def _log_debug(self) -> None:
        rospy.logdebug(
            "MuJoCo | pos=(%.3f, %.3f, %.3f) | motor_cmd=%s",
            self._odom_msg.pose.pose.position.x,
            self._odom_msg.pose.pose.position.y,
            self._odom_msg.pose.pose.position.z,
            np.array2string(self._current_motor_cmd, precision=3),
        )

    # ------------------------------------------------------------------
    def spin(self) -> None:
        """Start simulation loop."""
        use_viewer = self._enable_viewer and mj_viewer is not None
        if use_viewer:
            rospy.loginfo("Launching MuJoCo viewer.")
            with mj_viewer.launch_passive(self._model, self._data) as sim_viewer:
                self._loop(sim_viewer)
        else:
            if self._enable_viewer and mj_viewer is None:
                rospy.logwarn("MuJoCo viewer unavailable; running headless.")
            self._loop(None)

    def _loop(self, sim_viewer: Optional[object]) -> None:
        dt = self._physics_dt
        while not rospy.is_shutdown():
            start = time.time()
            mujoco.mj_step(self._model, self._data)
            if sim_viewer is not None and sim_viewer.is_running():
                sim_viewer.sync()
            sleep_time = dt - (time.time() - start)
            if sleep_time > 0:
                time.sleep(sleep_time)


def main() -> None:
    rospy.init_node("quad_sim_ros")
    bridge = QuadSimBridge()
    bridge.spin()


if __name__ == "__main__":
    main()
