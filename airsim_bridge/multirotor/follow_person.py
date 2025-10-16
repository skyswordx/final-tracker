# -*- coding: utf-8 -*-
import airsim
import time
import pprint
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

class PIDController:
    def __init__(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.previous_error = 0
        self.integral = 0
        
    def reset(self):
        self.previous_error = 0
        self.integral = 0
        
    def calculate(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

pid_x = PIDController(5, 0.0, 1)
pid_y = PIDController(5, 0.0, 1)
pid_z = PIDController(5, 0.0, 1)
pid_yaw = PIDController(5.0, 0.0, 1) 
#"192.168.110.186"
client = airsim.MultirotorClient(ip="")
#client = airsim.MultirotorClient(ip="192.168.110.186")
client.confirmConnection()
client.enableApiControl(True)

print("Arming the drone...")
client.armDisarm(True)

target_height_above_person = 0  # 50cm above the person's head

# 绘图数据初始化
x_data, y_data, z_data, time_data = [], [], [], []

# 设置绘图
plt.ion()  # 开启交互模式
fig, ax = plt.subplots()
ax.set_xlabel('Time (s)')
ax.set_ylabel('Velocity')
line_x, = ax.plot(time_data, x_data, label='X Velocity')
line_y, = ax.plot(time_data, y_data, label='Y Velocity')
line_z, = ax.plot(time_data, z_data, label='Z Velocity')
plt.legend()
plt.show()

start_time = time.time()

display_time_window = 10  # 显示最近10秒的数据

while True:
    current_time = time.time() - start_time
    person1_pose = client.simGetObjectPose("Person1")
    drone_pose = client.simGetVehiclePose()

    error_x = person1_pose.position.x_val - drone_pose.position.x_val
    error_y = person1_pose.position.y_val - drone_pose.position.y_val

    target_z = person1_pose.position.z_val + target_height_above_person
    error_z = target_z - drone_pose.position.z_val

    # 计算控制信号并限幅
    control_signal_x = max(min(pid_x.calculate(error_x, 0.02), 20), -20)
    control_signal_y = max(min(pid_y.calculate(error_y, 0.02), 20), -20)
    control_signal_z = max(min(pid_z.calculate(error_z, 0.02), 20), -20)

    # 计算无人机和人物之间的相对方向
    dx = person1_pose.position.x_val - drone_pose.position.x_val
    dy = person1_pose.position.y_val - drone_pose.position.y_val
    target_yaw = np.arctan2(dy, dx)  # 目标偏航角，弧度
    
    # 获取无人机当前的偏航角
    current_yaw = airsim.to_eularian_angles(drone_pose.orientation)[2]  # 取欧拉角中的偏航（yaw）
    
    # 计算偏航角的误差
    yaw_error = target_yaw - current_yaw
    # 确保误差在-pi到pi之间
    yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi
    
    # 使用PID控制器计算偏航速度控制信号
    control_signal_yaw = pid_yaw.calculate(yaw_error, 0.02)
    # 测试 机械臂角度控制
    try:
        # 如果你的 Python 客户端已经添加了 simSetSpecialValue(value, index, vehicle_name='')
        ok1 = client.simSetSpecialValue(30.0, 1, vehicle_name='')
        ok2 = client.simSetSpecialValue(60.0, 2, vehicle_name='')
        print(f"simSetSpecialValue(30, 1) -> {ok1}")
        print(f"simSetSpecialValue(60, 2) -> {ok2}")
    except Exception as e:
    # 兼容旧服务端或未注册 RPC 的情况
        print("simSetSpecialValue call failed (server may not support it yet):", e)
    
    # 更新绘图数据
    x_data.append(control_signal_x)
    y_data.append(control_signal_y)
    z_data.append(control_signal_z)
    time_data.append(current_time)

    # 保留最近10秒的数据
    while time_data and time_data[0] < current_time - display_time_window:
        time_data.pop(0)
        x_data.pop(0)
        y_data.pop(0)
        z_data.pop(0)

    # 更新绘图
    line_x.set_data(time_data, x_data)
    line_y.set_data(time_data, y_data)
    line_z.set_data(time_data, z_data)
    ax.set_xlim(current_time - display_time_window, current_time)  # 设置x轴显示的范围为最近10秒
    ax.relim()  # 重新计算限制
    ax.autoscale_view()  # 自动缩放
    fig.canvas.draw()
    fig.canvas.flush_events()

    #print(f"Person1 - Position: {pprint.pformat(person1_pose.position)}")
    #print(f"Person1 - orientation: {pprint.pformat(person1_pose.orientation)}")
    #print(f"Drone - Position: {pprint.pformat(drone_pose.position)}")
    #print(f"Drone - orientation: {pprint.pformat(drone_pose.orientation)}")
    #print(f"Control Signals - X: {control_signal_x}, Y: {control_signal_y}, Z: {control_signal_z}")

    # 使用限幅后的速度值发送控制命令
    client.moveByVelocityAsync(control_signal_x, control_signal_y, control_signal_z, 0.025, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(True, control_signal_yaw))

    time.sleep(0.02)
