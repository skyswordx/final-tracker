# 20250220 Wakkk Modified for ROS Noetic
import os

import mujoco
import mujoco.viewer as viewer
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray
import time

# 系统参数
gravity = 9.8066
mass = 2.0
dt = 0.005  # dt = model.opt.timestep
Ct = 6.450e-6
Cm = 1.056e-7
max_rot_velocity = 1434.7 

# 全局缓冲
odom = Odometry()


class ROSHandler:
    def __init__(self):
        rospy.init_node('quad_sim_ros')
        # 发布器
        self.odom_pub = rospy.Publisher('/quad/odometry', Odometry, queue_size=10)

        # 订阅电机命令
        self.motor_sub = rospy.Subscriber('/quad/motor_cmd', Float64MultiArray, self.motor_cb)
        self.current_motor_cmd = [0.0] * 4

    def motor_cb(self, msg):
        # 四个电机输入裁剪到 [0,1]
        self.current_motor_cmd = list(np.clip(msg.data, 0.0, 1.0))

    def publish_odometry(self, d):
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world"
        # 位置
        odom.pose.pose.position.x = d.qpos[0]
        odom.pose.pose.position.y = d.qpos[1]
        odom.pose.pose.position.z = d.qpos[2]
        # 姿态四元数，MuJoCo: [w,x,y,z] → ROS: [x,y,z,w]
        odom.pose.pose.orientation.x = d.sensordata[7]
        odom.pose.pose.orientation.y = d.sensordata[8]
        odom.pose.pose.orientation.z = d.sensordata[9]
        odom.pose.pose.orientation.w = d.sensordata[6]
        # 速度
        odom.twist.twist.linear = Vector3(d.qvel[0], d.qvel[1], d.qvel[2])
        odom.twist.twist.angular = Vector3(d.qvel[3], d.qvel[4], d.qvel[5])
        self.odom_pub.publish(odom)



ros_handler = ROSHandler()
log_count = 0  # 调试用计数


def control_callback(model, data):
    global log_count
    # 发布里程计
    ros_handler.publish_odometry(data)

    # 电机控制
    for i in range(4):
        data.actuator(f'motor{i+1}').ctrl[0] = ros_handler.current_motor_cmd[i]
    # 反算角速度并写回
    thrusts = [max(cmd, 0.0) * 12.5 for cmd in ros_handler.current_motor_cmd]
    motor_omegas = [np.sqrt(T / Ct) for T in thrusts]
    data.actuator('fl_motor').ctrl[0] = motor_omegas[0]
    data.actuator('fr_motor').ctrl[0] = motor_omegas[1]
    data.actuator('bl_motor').ctrl[0] = motor_omegas[2]
    data.actuator('br_motor').ctrl[0] = motor_omegas[3]

    # 调试打印
    log_count += 1
    if log_count >= 200:
        print("\n=== ODOMETRY DEBUG ===")
        print(f"Pos: {odom.pose.pose.position}")
        print(f"Ori: {odom.pose.pose.orientation}")
        print(f"Vel: {odom.twist.twist.linear}, AngVel: {odom.twist.twist.angular}")
        print(f"MotorCmd: {ros_handler.current_motor_cmd}")
        log_count = 0


if __name__ == '__main__':
    scene_path = os.path.join(os.path.dirname(__file__), 'scene', 'scene_quad.xml')
    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)
    mujoco.set_mjcb_control(control_callback)

    with viewer.launch_passive(model, data) as sim_viewer:
        while sim_viewer.is_running() and not rospy.is_shutdown():
            t0 = time.time()
            mujoco.mj_step(model, data)

            sim_viewer.sync()
            to_sleep = dt - (time.time() - t0)
            if to_sleep > 0:
                time.sleep(to_sleep)
