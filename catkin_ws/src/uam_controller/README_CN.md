# UAM Controller 软件包详细注释文档

## 概述
这是一个用于四旋翼无人机的级联式PID控制器ROS软件包，实现了从位置控制到姿态控制再到电机输出的完整控制链路。

---

## 文件结构说明

### 1. controller.hpp (头文件)
**路径**: `include/uam_controller/controller.hpp`

**作用**: 
- 定义了控制器节点的类结构和所有成员变量
- 包含姿态控制增益、位置控制增益、飞行器状态、设定点等数据结构
- 声明了所有控制算法的函数接口

**主要数据结构**:

#### AttitudeGains (姿态控制增益)
```cpp
struct AttitudeGains {
  double kp_roll;   // 滚转角比例增益
  double kp_pitch;  // 俯仰角比例增益
  double kp_yaw;    // 偏航角比例增益
  double ki_roll;   // 滚转角积分增益
  double ki_pitch;  // 俯仰角积分增益
  double ki_yaw;    // 偏航角积分增益
  double kd_roll;   // 滚转角微分增益 (实际用于角速度反馈)
  double kd_pitch;  // 俯仰角微分增益
  double kd_yaw;    // 偏航角微分增益
};
```

#### PositionGains (位置控制增益)
```cpp
struct PositionGains {
  double kp_xy;  // 水平位置比例增益
  double kp_z;   // 垂直位置比例增益
  double kd_xy;  // 水平速度微分增益
  double kd_z;   // 垂直速度微分增益
};
```

#### VehicleState (飞行器状态)
```cpp
struct VehicleState {
  ros::Time stamp;                    // 时间戳
  Eigen::Vector3d position;           // 位置 (世界坐标系)
  Eigen::Vector3d velocity;           // 速度 (世界坐标系)
  Eigen::Vector3d filtered_velocity;  // 滤波后的速度
  Eigen::Vector3d angular_velocity;   // 角速度 (机体坐标系)
  tf2::Quaternion orientation;        // 姿态四元数
  bool valid;                         // 状态是否有效
};
```

#### Setpoint (控制目标)
```cpp
struct Setpoint {
  Eigen::Vector3d position;  // 目标位置
  Eigen::Vector3d velocity;  // 目标速度 (当前未使用)
  double yaw;                // 目标偏航角
  bool has_yaw;              // 是否包含偏航角指令
  ros::Time stamp;           // 时间戳
  bool valid;                // 设定点是否有效
};
```

**主要成员变量**:
- `nh_`, `pnh_`: ROS节点句柄
- `odom_sub_`, `setpoint_sub_`, `disturbance_sub_`: 订阅器
- `motor_pub_`: 电机指令发布器
- `control_timer_`: 控制循环定时器
- `mass_`: 飞行器质量 (kg)
- `gravity_`: 重力加速度 (m/s²)
- `inertia_diag_`: 转动惯量对角线元素 (kg·m²)
- `rotor_arm_length_`: 每个旋翼的臂长 (m)
- `rotor_directions_`: 旋翼旋转方向 (+1顺时针, -1逆时针)
- `rotor_angles_`: 旋翼相对于机体x轴的角度 (rad)
- `mixer_matrix_`: 控制分配矩阵 (将推力和力矩映射到各电机推力)

---

### 2. controller_node.cpp (主控制器实现)
**路径**: `src/controller_node.cpp`

**作用**: 
实现了四旋翼的完整控制算法，包括位置控制、姿态控制和电机混控。

**核心函数详解**:

#### ControllerNode::ControllerNode()
**功能**: 构造函数，初始化控制器
- 加载参数
- 设置电机混控矩阵
- 订阅话题
- 创建控制定时器

#### loadParameters()
**功能**: 从ROS参数服务器加载所有配置参数
- 读取话题名称
- 读取物理参数 (质量、惯量、电机常数等)
- 读取控制增益
- 从全局配置加载旋翼配置 (`/uam_config/rotor_configuration`)
- 支持自定义X型/+型/混合型布局

**旋翼配置解析**:
- 通过角度和臂长确定旋翼在机体坐标系中的位置
- 根据位置象限 (前左/前右/后左/后右) 分配旋翼索引
- 计算力-力矩转换比率 kappa = moment_constant / motor_constant

#### setupMixer()
**功能**: 构建控制分配矩阵
```
控制分配方程: [F; τx; τy; τz] = A * [f1; f2; f3; f4]
其中:
- F: 总推力
- τx, τy, τz: 滚转/俯仰/偏航力矩
- f1~f4: 四个电机的推力

分配矩阵 A:
A(0,i) = 1              // 所有电机贡献推力
A(1,i) = -y_i           // y方向位置产生滚转力矩
A(2,i) = x_i            // x方向位置产生俯仰力矩
A(3,i) = dir_i * κ_i   // 旋转方向产生偏航力矩

混控矩阵 = A的逆矩阵
```

#### odomCallback()
**功能**: 接收里程计数据，更新飞行器状态
- 提取位置、速度、姿态、角速度
- 对速度进行低通滤波: `v_filtered = α*v + (1-α)*v_filtered_prev`
- 发布调试话题 (如果启用)

#### setpointCallback()
**功能**: 接收位置设定点
- 提取目标位置
- 从四元数中提取偏航角 (roll和pitch会被忽略，由位置控制器自动生成)

#### disturbanceCallback()
**功能**: 接收外部扰动估计 (由Neural-Fly等算法提供)
- 存储扰动力向量 (机体坐标系)
- 记录时间戳用于判断数据是否新鲜

#### controlLoop()
**功能**: 主控制循环 (200Hz)

**控制流程**:
1. **检查使能**: 如果未使能，输出零油门并返回
2. **位置控制**: 
   - 计算位置误差: `e_pos = pos_target - pos_current`
   - 计算期望速度: `v_des = Kp_pos * e_pos`
   - 限制垂直速度在 [max_descent_rate, max_ascent_rate] 范围内
   - 计算速度误差: `e_vel = v_des - v_current`
   - 计算期望加速度: `a_des = Kd_pos * e_vel` (实际上Kd_pos是速度环比例增益)
   - 限制水平加速度以避免过度倾斜

3. **力映射到姿态**:
   - 计算总合力: `F = m*a_des + m*g - F_disturbance`
   - 期望z轴方向 (机体): `z_b = F / |F|`
   - 根据期望偏航角构造期望x轴
   - 用叉乘构造完整旋转矩阵，得到期望姿态四元数
   - 期望推力: `T = |F|`

4. **姿态控制** (PID):
   - 计算姿态误差四元数: `q_err = q_current^(-1) * q_des`
   - 转换为角度误差: `e_att = 2 * [qx, qy, qz]`
   - 积分项更新: `∫e_att += e_att * dt` (带限幅)
   - 计算角加速度指令:
     ```
     α_roll  = Kp_roll  * e_roll  + Kd_roll  * (-ω_x) + Ki_roll  * ∫e_roll
     α_pitch = Kp_pitch * e_pitch + Kd_pitch * (-ω_y) + Ki_pitch * ∫e_pitch
     α_yaw   = Kp_yaw   * e_yaw   + Kd_yaw   * (-ω_z) + Ki_yaw   * ∫e_yaw
     ```
   - 计算期望力矩: `τ = I * α` (I是转动惯量对角矩阵)

5. **控制分配**:
   - 使用混控矩阵: `[f1; f2; f3; f4] = mixer_matrix * [T; τx; τy; τz]`
   - 将推力转换为油门指令: `throttle = f / motor_force_scale`
   - 限制在 [motor_min_cmd, motor_max_cmd] 范围内

6. **发布电机指令**

#### computePositionControl()
**功能**: 实现位置到加速度的串级PID控制
- 外环: 位置比例控制 -> 期望速度
- 内环: 速度比例控制 -> 期望加速度
- 包含速度限制和倾斜角限制

#### computeOrientationTargets()
**功能**: 从力向量计算期望姿态
- 使用力方向作为期望z轴
- 结合期望偏航角构造完整姿态
- 返回期望四元数和推力大小

#### mixWrenchToThrust()
**功能**: 控制分配 - 将力和力矩分解到四个电机
- 应用混控矩阵
- 限制负推力为零

#### publishMotorCommand()
**功能**: 发布归一化的电机指令 [0, 1]
- 按照 motor_output_order 映射输出
- 发布到 `/quad/motor_cmd` 话题

---

### 3. setpoint_publisher.cpp (轨迹发布器)
**路径**: `src/setpoint_publisher.cpp`

**作用**: 
生成参考轨迹供控制器跟踪，支持多种轨迹模式。

**轨迹类型**:

#### 1. hover (悬停)
- 在指定位置 `hover_point` 保持静止
- 适合测试定点悬停性能

#### 2. circle (圆形轨迹)
- 参数:
  - `circle_radius`: 圆半径 (m)
  - `circle_period`: 运动周期 (s)
- 轨迹方程:
  ```
  x(t) = x0 + r * cos(ω*t)
  y(t) = y0 + r * sin(ω*t)
  z(t) = z0
  其中 ω = 2π / T
  ```
- 偏航角自动对准运动方向: `yaw = atan2(dy/dt, dx/dt)`

#### 3. line (直线往返)
- 参数:
  - `line_length`: 直线长度 (m)
  - `line_speed`: 运动速度 (m/s)
- 沿x轴往返运动
- 位置: `x(t) = x0 + offset`, 其中offset在 [-L/2, L/2] 范围内往返

**发布频率**: 20Hz (可配置)

---

### 4. debug_macros.hpp (调试宏)
**路径**: `include/uam_controller/debug_macros.hpp`

**作用**: 
提供分级的调试输出和数据发布功能。

**调试级别**:
- `UAM_DEBUG_LEVEL_NONE (0)`: 无调试输出
- `UAM_DEBUG_LEVEL_FLOW (1)`: 输出控制流程信息
- `UAM_DEBUG_LEVEL_DETAIL (2)`: 输出详细调试信息

**编译时设置**:
```bash
catkin build uam_controller --cmake-args -DUAM_DEBUG_LEVEL=2
```

**调试宏**:
- `UAM_DEBUG_FLOW(msg)`: 输出流程信息 (1秒限流)
- `UAM_DEBUG_DETAIL(msg)`: 输出详细信息 (1秒限流)
- `UAM_DEBUG_PUBLISH_SCALAR(ptr, topic, value)`: 发布标量调试数据
- `UAM_DEBUG_PUBLISH_VECTOR(ptr, topic, vec)`: 发布向量调试数据

**限流机制**: 使用哈希表记录每个调试点的上次输出时间，避免输出过于频繁

---

### 5. log_publisher.hpp (日志发布器)
**路径**: `include/uam_controller/log_publisher.hpp`

**作用**: 
动态创建ROS话题发布调试数据，便于实时可视化。

**功能**:
- `publishScalar()`: 发布单个浮点数 (std_msgs::Float64)
- `publishVector()`: 发布三维向量 (geometry_msgs::Vector3)

**话题自动创建**: 首次发布到某个话题时自动创建发布器

**使用场景**:
- 在rqt_plot中实时查看控制误差、指令值等
- 调试PID参数时观察响应曲线

**发布的调试话题** (当 DEBUG_LEVEL >= 2 时):
```
/uam_controller/debug/position          # 当前位置
/uam_controller/debug/velocity          # 当前速度
/uam_controller/debug/angular_velocity  # 当前角速度
/uam_controller/debug/target_position   # 目标位置
/uam_controller/debug/disturbance       # 扰动估计
/uam_controller/debug/position_error    # 位置误差
/uam_controller/debug/accel_cmd         # 加速度指令
/uam_controller/debug/attitude_error    # 姿态误差
/uam_controller/debug/ang_acc_cmd       # 角加速度指令
/uam_controller/debug/torque_cmd        # 力矩指令
/uam_controller/debug/throttle_0~3      # 各电机油门
```

---

## 配置文件详解

### 1. uam_config.yaml (飞行器物理参数)
**路径**: `config/uam_config.yaml`

**作用**: 定义飞行器的物理特性和电机配置

#### 全局参数
```yaml
mode_exp: 1              # 运行模式 (1=仿真, 2=调试, 3=实验)
max_thrust: 50.0         # 最大总推力 (N)
```

#### 机体参数
```yaml
mass_quad_body: 2.458    # 机身质量 (kg), 不含旋翼
inertia_quad_body:       # 转动惯量张量 (kg·m²)
  xx: 0.01860            # Ixx - 绕x轴转动惯量
  xy: 0.0                # Ixy - 惯性积 (对称机体为0)
  xz: 0.0
  yy: 0.01860            # Iyy - 绕y轴转动惯量
  yz: 0.0
  zz: 0.03215            # Izz - 绕z轴转动惯量 (通常最大)

config_quad_body:
  body_width: 0.3        # 机身宽度 (m)
  body_height: 0.16      # 机身高度 (m)
  arm_length: 0.225      # 机臂长度 (m), 从中心到旋翼轴心
```

**惯量说明**:
- Ixx, Iyy: 横滚/俯仰惯量，影响姿态响应速度
- Izz: 偏航惯量，通常约为 Ixx + Iyy
- 对称机体的惯性积应为零

#### 电机/旋翼参数
```yaml
mass_rotor: 0.005                # 单个旋翼质量 (kg)
rotor_offset_top: 0.023          # 旋翼相对机身的垂直偏移 (m)
radius_rotor: 0.1                # 旋翼半径 (m)

motor_constant: 6.450e-6         # 电机拉力常数 Ct (kg·m/s²)
                                 # 推力公式: F = ω² * Ct
                                 # 其中 ω 是角速度 (rad/s)

moment_constant: 1.056e-7        # 力矩常数 Cm (m)
                                 # 反扭矩: τ = ω² * Cm

time_constant_up: 0.0125         # 电机加速时间常数 (s)
time_constant_down: 0.025        # 电机减速时间常数 (s)
                                 # 模拟电机响应延迟

max_rot_velocity: 1434.7         # 最大旋转速度 (rad/s)
                                 # 对应约 13699 RPM

rotor_drag_coefficient: 8.06428e-05    # 风阻系数
rolling_moment_coefficient: 0.000001   # 滚转力矩系数
```

**电机常数标定方法**:
1. 测量悬停时电机转速和总质量
2. 由 `4 * Ct * ω²_hover = m * g` 反推 Ct
3. 测量偏航力矩响应，反推 Cm

#### 旋翼配置 (X型布局)
```yaml
rotor_configuration:
  '0':  # 前左电机 (motor 1)
    angle: 0.78539              # 45° = π/4 rad
    arm_length: 0.225           # 机臂长度 (m)
    rotor_force_constant: 6.450e-6
    rotor_moment_constant: 1.056e-7
    direction: -1.0             # 逆时针旋转

  '1':  # 前右电机 (motor 2)
    angle: -0.78539             # -45° = -π/4 rad
    direction: 1.0              # 顺时针

  '2':  # 后左电机 (motor 3)
    angle: 2.35619              # 135° = 3π/4 rad
    direction: 1.0

  '3':  # 后右电机 (motor 4)
    angle: -2.35619             # -135° = -3π/4 rad
    direction: -1.0
```

**坐标系定义**:
- x轴: 指向机头 (前)
- y轴: 指向左侧
- z轴: 向上
- 角度: 从x轴逆时针测量 (俯视图)

**电机编号约定**:
```
         x (前)
         ↑
    1(↺) │ 2(↻)
         │
  ───────┼───────→ y (左)
         │
    3(↻) │ 4(↺)
```
- ↺: 逆时针 (direction = -1)
- ↻: 顺时针 (direction = +1)

---

### 2. controller_params.yaml (控制器参数)
**路径**: `config/controller_params.yaml`

#### 话题配置
```yaml
frame_id: "world"                          # 世界坐标系名称
odom_topic: "/quad/odometry"               # 里程计输入
setpoint_topic: "/quad/position_setpoint"  # 位置设定点输入
disturbance_topic: "/quad/disturbance_estimate"  # 扰动估计输入
motor_cmd_topic: "/quad/motor_cmd"         # 电机指令输出
```

#### 控制器基础参数
```yaml
control_rate: 200.0            # 控制频率 (Hz)
                               # 建议 >= 100Hz 以保证稳定性

mass: 2.458                    # 总质量 (kg), 含电池/载荷
gravity: 9.8066                # 重力加速度 (m/s²)
max_thrust: 50.0               # 最大总推力 (N)
hover_throttle: 0.5            # 悬停油门估计值 (0-1)
                               # 用于前馈, 应接近 m*g / max_thrust
```

#### 电机参数 (与uam_config.yaml对应)
```yaml
arm_length: 0.225              # 机臂长度 (m)
motor_constant: 6.450e-6       # 拉力常数
moment_constant: 1.056e-7      # 力矩常数

inertia_xx: 0.01860            # 转动惯量 (kg·m²)
inertia_yy: 0.01860
inertia_zz: 0.03215

motor_time_constant_up: 0.0125      # 电机时间常数 (s)
motor_time_constant_down: 0.025

rotor_directions: [-1.0, 1.0, 1.0, -1.0]  # 旋翼转向
                                           # 顺序: [前左, 前右, 后左, 后右]
```

#### 控制限制
```yaml
motor_force_scale: 12.5        # 推力到油门的缩放因子
                               # throttle = force / scale
                               # 应设置为单电机最大推力的估计值

motor_min_cmd: 0.0             # 最小油门 (0-1)
motor_max_cmd: 0.95            # 最大油门 (0-1)
                               # 留有余量避免饱和

max_tilt_deg: 35.0             # 最大倾斜角 (度)
                               # 限制激进机动，保证安全

max_ascent_rate: 2.0           # 最大上升速度 (m/s)
max_descent_rate: 1.0          # 最大下降速度 (m/s)
```

#### 滤波器参数
```yaml
velocity_filter_alpha: 0.35    # 速度低通滤波系数 (0-1)
                               # v_filt = α*v + (1-α)*v_filt_prev
                               # 越大响应越快，但噪声越大

integral_limit: 0.2            # 积分限幅 (rad)
                               # 防止积分饱和

feedforward_timeout: 0.2       # 扰动估计超时 (s)
                               # 超过此时间未更新则不使用
```

#### 位置控制增益
```yaml
position_kp_xy: 1.2            # 水平位置比例增益 (1/s)
                               # 位置误差 -> 速度指令
                               # v_des = Kp * (pos_target - pos_current)

position_kp_z: 6.0             # 垂直位置比例增益 (1/s)
                               # 通常设置比xy大，因为z轴更直接

velocity_kd_xy: 1.5            # 水平速度增益 (1/s)
                               # 速度误差 -> 加速度指令
                               # a_des = Kd * (v_des - v_current)

velocity_kd_z: 3.0             # 垂直速度增益 (1/s)
```

**位置环调参建议**:
1. 从小的Kp开始 (如0.5), 逐渐增大直到响应满意
2. Kd应约为Kp的1-2倍
3. z轴增益可比xy大2-5倍
4. 过大的Kp会导致振荡

#### 姿态控制增益
```yaml
attitude_kp_roll: 6.0          # 滚转角比例增益
attitude_kp_pitch: 6.0         # 俯仰角比例增益
attitude_kp_yaw: 6.0           # 偏航角比例增益

attitude_ki_roll: 0.0          # 滚转角积分增益
attitude_ki_pitch: 0.0         # 俯仰角积分增益
attitude_ki_yaw: 0.0           # 偏航角积分增益
                               # 通常设为0，除非有持续性偏差

attitude_kd_roll: 1.2          # 滚转角速度增益
attitude_kd_pitch: 1.2         # 俯仰角速度增益
attitude_kd_yaw: 1.2           # 偏航角速度增益
```

**姿态环调参建议**:
1. Roll/Pitch通常使用相同增益 (对称机体)
2. Kp越大响应越快，但过大会振荡
3. Kd提供阻尼，应约为Kp的10-30%
4. Yaw可以更慢 (增益更小)，不影响主要飞行性能
5. 积分项谨慎使用，容易引起超调

**典型增益范围**:
- 小型四旋翼 (0.5kg): Kp=3-6, Kd=0.5-1.5
- 中型四旋翼 (2.5kg): Kp=5-10, Kd=1-2
- 大型四旋翼 (10kg): Kp=2-4, Kd=0.5-1

#### 高级功能
```yaml
use_yaw_rate_stabilizer: true  # 是否使用偏航角速度反馈
                               # true: 阻尼偏航振荡
                               # false: 仅角度反馈
```

---

### 轨迹发布器参数
```yaml
setpoint_publisher:
  publish_rate: 20.0           # 发布频率 (Hz)
  start_delay: 1.0             # 启动延迟 (s)
  trajectory: "hover"          # 轨迹类型: hover/circle/line
  hover_point: [0.0, 0.0, 1.0] # 悬停点 [x, y, z] (m)
  path_frame_id: "world"       # 坐标系
  setpoint_topic: "/quad/position_setpoint"

  # 圆形轨迹参数
  circle_radius: 1.0           # 半径 (m)
  circle_period: 20.0          # 周期 (s), 角速度 = 2π/T

  # 直线轨迹参数
  line_speed: 0.5              # 速度 (m/s)
  line_length: 2.0             # 长度 (m)
```

---

## 控制器工作原理总结

### 控制架构 (级联PID)
```
位置设定点 -> [位置控制器] -> 期望速度 -> [速度控制器] -> 期望加速度
              ↓
         期望姿态+推力 <- [力到姿态映射]
              ↓
       [姿态控制器] -> 期望角加速度 -> 期望力矩
              ↓
       [控制分配] -> 四个电机推力指令
```

### 坐标系
- **世界坐标系** (NED或ENU): 位置、速度控制在此进行
- **机体坐标系**: 力、力矩、角速度在此表示
  - x轴: 前 (机头方向)
  - y轴: 右 (机翼方向) 或 左 (取决于约定)
  - z轴: 下 (推力反方向) 或 上

### 关键公式

#### 1. 位置控制
```
e_pos = pos_target - pos_current
v_des = Kp_pos * e_pos
e_vel = v_des - v_current
a_des = Kd_vel * e_vel
```

#### 2. 力到姿态
```
F_total = m*a_des + m*g - F_disturbance
z_b_des = F_total / |F_total|
T_des = |F_total|
```

#### 3. 姿态控制
```
q_err = q_current^(-1) * q_des
e_att = 2 * [qx, qy, qz]
α = Kp*e_att - Kd*ω + Ki*∫e_att
τ = I * α
```

#### 4. 控制分配 (X型四旋翼)
```
[f1]       [1    1    1    1  ]^(-1)  [T  ]
[f2] = 1/4*[1   -1   -1    1  ]     * [τx ]
[f3]       [1   -1    1   -1  ]       [τy ]
[f4]       [1    1   -1   -1  ]       [τz ]

(简化形式，实际需考虑机臂长度和kappa系数)
```

---

## 使用示例

### 1. 启动控制器
```bash
roslaunch uam_controller uam_controller.launch
```

### 2. 使能起飞
```bash
rosrun dynamic_reconfigure dynparam set /uam_controller_node enable_takeoff true
```

### 3. 发布位置指令
```bash
rostopic pub /quad/position_setpoint geometry_msgs/PoseStamped "
header:
  frame_id: 'world'
pose:
  position: {x: 1.0, y: 0.5, z: 1.5}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

### 4. 查看调试信息 (需编译时启用DEBUG)
```bash
rqt_plot /uam_controller/debug/position/x:y:z /uam_controller/debug/target_position/x:y:z
```

### 5. 动态调参
```bash
rosrun rqt_reconfigure rqt_reconfigure
# 选择 uam_controller_node 节点进行参数调整
```

---

## 故障排查

### 问题1: 飞行器不响应控制指令
**可能原因**:
- `enable_takeoff` 未设置为true
- 未收到里程计数据 (`odom_topic`)
- 未收到设定点 (`setpoint_topic`)

**检查**:
```bash
rostopic echo /quad/odometry         # 检查里程计
rostopic echo /quad/position_setpoint # 检查设定点
rosrun dynamic_reconfigure dynparam get /uam_controller_node enable_takeoff
```

### 问题2: 飞行器振荡
**可能原因**:
- 控制增益过大
- 位置/姿态估计有噪声
- 控制频率不足

**解决方法**:
1. 降低Kp增益 (位置/姿态)
2. 增大速度滤波系数 `velocity_filter_alpha`
3. 检查传感器数据质量
4. 确保控制频率 >= 100Hz

### 问题3: 飞行器倾斜/偏航漂移
**可能原因**:
- 转动惯量设置不准确
- 电机常数标定有误
- 混控矩阵错误

**解决方法**:
1. 重新测量/计算转动惯量
2. 标定电机推力常数
3. 验证旋翼配置 (`rotor_configuration`)
4. 增加姿态积分增益 Ki (小心使用)

### 问题4: 高度控制不准
**可能原因**:
- 质量参数不准
- 悬停油门 `hover_throttle` 设置有误
- 推力常数标定不准

**解决方法**:
1. 精确测量总质量
2. 调整 `hover_throttle` 接近 m*g / max_thrust
3. 增大 z轴的Kp增益

---

## 扩展功能

### 与Neural-Fly集成
控制器支持接收外部扰动估计 (`/quad/disturbance_estimate`)，可与Neural-Fly等学习算法结合:

1. Neural-Fly实时估计扰动力
2. 通过 `disturbance_topic` 发送给控制器
3. 控制器将扰动作为前馈补偿项
4. 提高抗扰性能

**使能扰动补偿**:
```bash
rosrun dynamic_reconfigure dynparam set /uam_controller_node enable_feedforward true
```

---

## 参考文献

1. **Geometric Tracking Control of a Quadrotor UAV**  
   Lee et al., CDC 2010
   
2. **Minimum Snap Trajectory Generation**  
   Mellinger and Kumar, ICRA 2011

3. **Neural-Fly: Learning-based Adaptive Control**  
   O'Connell et al., Science Robotics 2022

---

## 维护日志

- **版本**: 1.0
- **最后更新**: 2025-01-13
- **作者**: UAM Lab
- **许可**: MIT

---

如有问题，请提交Issue或联系维护团队。
