import airsim
import time
import keyboard

# 初始化AirSim客户端
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# 控制参数配置
MOVE_SPEED = 3.0      # 水平移动速度 (m/s)
VERTICAL_SPEED = 2.0  # 垂直速度 (m/s)
YAW_RATE = 45.0       # 偏航角速度 (度/秒)

# 初始化控制状态
current_vx = 0.0
current_vy = 0.0
current_vz = 0.0
current_yaw = airsim.YawMode(is_rate=True, yaw_or_rate=0.0)

def update_controls():
    global current_vx, current_vy, current_vz, current_yaw
    
    # 重置控制量
    current_vx = 0.0
    current_vy = 0.0
    current_vz = 0.0
    current_yaw.yaw_or_rate = 0.0

    # 水平移动控制 (WASD)
    if keyboard.is_pressed('w'):
        current_vx = MOVE_SPEED   # 前向 (X轴)
    if keyboard.is_pressed('s'):
        current_vx = -MOVE_SPEED  # 后向
    if keyboard.is_pressed('a'):
        current_vy = -MOVE_SPEED  # 左移 (Y轴)
    if keyboard.is_pressed('d'):
        current_vy = MOVE_SPEED   # 右移

    # 垂直运动控制 (上下箭头)
    if keyboard.is_pressed('up'):
        current_vz = -VERTICAL_SPEED  # 上升 (AirSim Z轴向下为正)
    if keyboard.is_pressed('down'):
        current_vz = VERTICAL_SPEED   # 下降

    # 偏航控制 (左右箭头)
    if keyboard.is_pressed('left'):
        current_yaw.yaw_or_rate = YAW_RATE   # 左转 (逆时针)
    if keyboard.is_pressed('right'):
        current_yaw.yaw_or_rate = -YAW_RATE  # 右转 (顺时针)

try:
    print("按 ESC 退出控制")
    client.takeoffAsync().join()  # 起飞

    while True:
        if keyboard.is_pressed('esc'):
            break

        update_controls()
        
        # 发送速度控制指令（持续0.1秒）
        client.moveByVelocityAsync(
            vx = current_vx,
            vy = current_vy,
            vz = current_vz,
            duration = 0.1,
            yaw_mode = current_yaw,
            drivetrain = airsim.DrivetrainType.MaxDegreeOfFreedom
        )
        
        time.sleep(0.05)

finally:
    print("正在着陆...")
    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)