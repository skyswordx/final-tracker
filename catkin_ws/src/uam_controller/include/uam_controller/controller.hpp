#pragma once

#include <array>
#include <cmath>
#include <memory>
#include <string>

// ROS 和相关库的头文件
#include <dynamic_reconfigure/server.h> // 动态调参服务器
#include <geometry_msgs/PoseStamped.h> // 位姿消息（用于目标点）
#include <geometry_msgs/Vector3Stamped.h> // 带时间戳的三维向量（用于扰动）
#include <nav_msgs/Odometry.h> // 里程计消息（用于无人机状态）
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h> // 浮点数数组（用于电机指令）
#include <tf2/LinearMath/Matrix3x3.h> // tf2 库，用于姿态表示
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Eigen 库，用于线性代数运算
#include <Eigen/Dense>

// 自定义头文件
#include <uam_controller/debug_macros.hpp> // 调试宏
#include <uam_controller/log_publisher.hpp> // 调试话题发布器
#include <uam_controller/uam_controllerConfig.h> // 动态调参生成的头文件

namespace uam_controller
{

/**
 * @brief 姿态环 PID 控制器增益
 */
struct AttitudeGains
{
  double kp_roll{6.0};    // 滚转角比例增益
  double kp_pitch{6.0};   // 俯仰角比例增益
  double kp_yaw{3.0};     // 偏航角比例增益
  double ki_roll{0.0};    // 滚转角积分增益
  double ki_pitch{0.0};   // 俯仰角积分增益
  double ki_yaw{0.2};     // 偏航角积分增益
  double kd_roll{0.15};   // 滚转角微分增益 (角速度阻尼)
  double kd_pitch{0.15};  // 俯仰角微分增益
  double kd_yaw{0.1};     // 偏航角微分增益
};

/**
 * @brief 位置环 PID 控制器增益
 */
struct PositionGains
{
  double kp_xy{3.0};  // 水平位置比例增益
  double kp_z{6.0};   // 垂直位置比例增益
  double kd_xy{3.5};  // 水平速度比例增益 (位置微分项)
  double kd_z{4.5};   // 垂直速度比例增益
};

/**
 * @brief 存储无人机当前状态的结构体
 */
struct VehicleState
{
  ros::Time stamp;                                        // 时间戳
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};      // 位置 (m)
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};      // 速度 (m/s)
  Eigen::Vector3d filtered_velocity{Eigen::Vector3d::Zero()}; // 滤波后的速度 (m/s)
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()}; // 角速度 (rad/s)
  tf2::Quaternion orientation{0.0, 0.0, 0.0, 1.0};        // 姿态四元数
  bool valid{false};                                      // 状态是否有效（是否已收到里程计数据）
};

/**
 * @brief 存储期望目标点的结构体
 */
struct Setpoint
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()}; // 期望位置
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()}; // 期望速度 (当前未使用)
  double yaw{0.0};                                   // 期望偏航角
  bool has_yaw{false};                               // 目标点是否包含有效的偏航角
  ros::Time stamp;                                   // 时间戳
  bool valid{false};                                 // 目标点是否有效
};

/**
 * @class ControllerNode
 * @brief 四旋翼无人机位置-姿态级联PID控制器
 *
 * 该类实现了无人机的核心控制逻辑。它订阅里程计信息和目标点，
 * 通过内外环控制器计算出四个电机的指令，并将其发布出去。
 */
class ControllerNode
{
public:
  ControllerNode();
  void spin();

private:
  // 定义动态调参服务器类型
  using ReconfigureServer = dynamic_reconfigure::Server<uam_controller::uam_controllerConfig>;

  // --- 初始化函数 ---
  void loadParameters();     // 从参数服务器加载所有配置
  void setupSubscriptions(); // 设置所有ROS话题的订阅者
  void setupMixer();         // 根据无人机配置计算电机混合矩阵

  // --- ROS 回调函数 ---
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void setpointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void disturbanceCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
  void reconfigureCallback(uam_controller::uam_controllerConfig &cfg, uint32_t level);
  void controlLoop(const ros::TimerEvent &event); // 主控制循环

  // --- 控制算法核心函数 ---
  Eigen::Vector3d computePositionControl(double dt, double &desired_yaw);
  void computeOrientationTargets(const Eigen::Vector3d &force_cmd,
                                 double yaw_des,
                                 tf2::Quaternion &q_des_out,
                                 double &thrust_out) const;
  Eigen::Vector4d mixWrenchToThrust(double thrust, const Eigen::Vector3d &torques) const;
  void publishMotorCommand(const Eigen::Vector4d &thrusts);
  void resetIntegrators();

  // --- 辅助函数 ---
  bool disturbanceFresh(const ros::Time &now) const;
  double clamp(double value, double min_val, double max_val) const;

  // --- ROS 相关成员 ---
  ros::NodeHandle nh_;      // 公共 ROS 节点句柄
  ros::NodeHandle pnh_{"~"}; // 私有 ROS 节点句柄 (用于参数)

  ros::Subscriber odom_sub_;
  ros::Subscriber setpoint_sub_;
  ros::Subscriber disturbance_sub_;
  ros::Publisher motor_pub_;

  ros::Timer control_timer_; // 控制循环定时器

  std::unique_ptr<ReconfigureServer> reconfig_server_; // 动态调参服务器

  // --- 话题与坐标系名称 ---
  std::string frame_id_{"world"};
  std::string odom_topic_{"/quad/odometry"};
  std::string setpoint_topic_{"/quad/position_setpoint"};
  std::string disturbance_topic_{"/quad/disturbance_estimate"};
  std::string motor_topic_{"/quad/motor_cmd"};

  // --- 物理与配置参数 ---
  double control_rate_hz_{200.0};       // 控制频率 (Hz)
  double mass_{2.458};                  // 质量 (kg)
  double gravity_{9.8066};              // 重力加速度 (m/s^2)
  double max_thrust_{50.0};             // 最大总推力 (N)
  double hover_throttle_{0.5};          // 悬停油门估计值
  double arm_length_{0.225};            // 力臂长度 (m)
  double motor_force_scale_{12.5};      // 推力到油门的转换系数
  double motor_constant_{6.450e-6};     // 电机拉力系数
  double moment_constant_{1.056e-7};    // 电机力矩系数
  Eigen::Vector3d inertia_diag_{0.01860, 0.01860, 0.03215}; // 转动惯量 (kg*m^2)
  
  // --- 电机布局参数 (默认X型四旋翼) ---
  // 顺序: 0:前左, 1:前右, 2:后左, 3:后右
  std::array<double, 4> rotor_directions_{{1.0, -1.0, -1.0, 1.0}}; // 旋翼旋转方向 (+1:CCW, -1:CW)
  std::array<Eigen::Vector2d, 4> rotor_xy_; // 各旋翼在机体坐标系下的XY位置
  std::array<double, 4> rotor_kappa_; // 力矩/推力系数比
  
  // --- 控制器内部参数与状态 ---
  double velocity_filter_alpha_{0.35};  // 速度低通滤波器系数
  double integral_limit_{0.2};          // 积分抗饱和阈值
  double disturbance_timeout_{0.2};     // 扰动估计超时时间 (s)
  double motor_min_cmd_{0.0};           // 最小油门
  double motor_max_cmd_{0.95};          // 最大油门
  double max_tilt_deg_{35.0};           // 最大倾斜角 (度)
  double max_ascent_rate_{2.0};         // 最大上升速度 (m/s)
  double max_descent_rate_{1.0};        // 最大下降速度 (m/s)
  bool use_yaw_rate_stabilizer_{true};  // 是否使用角速度稳定偏航

  PositionGains pos_gains_; // 位置环增益
  AttitudeGains att_gains_; // 姿态环增益

  VehicleState state_;      // 当前无人机状态
  Setpoint setpoint_;       // 期望目标点
  Eigen::Vector3d disturbance_force_{Eigen::Vector3d::Zero()}; // 扰动估计值
  ros::Time disturbance_stamp_;

  Eigen::Vector3d attitude_integral_{Eigen::Vector3d::Zero()}; // 姿态误差积分项

  double target_yaw_{0.0};        // 手动设定的目标偏航角 (当目标点未提供时使用)
  bool enable_takeoff_{false};    // 是否使能控制器输出 (起飞开关)
  bool enable_feedforward_{true}; // 是否启用扰动前馈补偿

  Eigen::Matrix4d mixer_matrix_; // 电机混合矩阵 (逆分配矩阵)

  ros::Time last_control_time_; // 上次控制循环的时间戳

#if UAM_DEBUG_LEVEL >= UAM_DEBUG_LEVEL_DETAIL
  std::unique_ptr<LogPublisher> log_pub_; // 调试话题发布器
#endif
};

}  // 命名空间 uam_controller 结束
