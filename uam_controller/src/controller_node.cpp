#include "uam_controller/controller.hpp"

#include <algorithm>
#include <cmath>

// 包含用于参数加载和动态调参的头文件
#include <XmlRpcValue.h>
#include <boost/bind/bind.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/QR>

using namespace boost::placeholders;

namespace uam_controller
{
namespace
{
// 角度到弧度的转换常数
constexpr double kDegToRad = M_PI / 180.0;
}

/**
 * @brief ControllerNode 构造函数
 */
ControllerNode::ControllerNode()
{
  loadParameters(); // 1. 加载所有参数
  setupMixer();     // 2. 设置电机混合矩阵
  setupSubscriptions(); // 3. 初始化 ROS 话题订阅

  // 如果调试等级足够高，则创建调试日志发布器
#if UAM_DEBUG_LEVEL >= UAM_DEBUG_LEVEL_DETAIL
  log_pub_ = std::make_unique<LogPublisher>(nh_);
#endif

  // 初始化电机指令发布者
  motor_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(motor_topic_, 10);

  // 设置动态调参服务器
  reconfig_server_ = std::unique_ptr<ReconfigureServer>(new ReconfigureServer(pnh_));
  ReconfigureServer::CallbackType cb = boost::bind(&ControllerNode::reconfigureCallback, this, _1, _2);
  reconfig_server_->setCallback(cb);

  // 创建一个定时器，以指定的频率调用 controlLoop 函数
  control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_hz_), &ControllerNode::controlLoop, this);

  UAM_DEBUG_FLOW("控制器初始化完成，已订阅里程计话题: " << odom_topic_);
}

/**
 * @brief 启动节点，并等待关闭
 */
void ControllerNode::spin()
{
  // 使用异步 Spinner，确保回调函数可以在独立的线程中处理，避免阻塞
  ros::AsyncSpinner spinner(2); // 使用2个线程
  spinner.start();
  ros::waitForShutdown();
}

/**
 * @brief 从 ROS 参数服务器加载所有参数
 */
void ControllerNode::loadParameters()
{
  // 从私有节点句柄 pnh_ 读取参数，允许通过 launch 文件进行命名空间隔离
  pnh_.param<std::string>("frame_id", frame_id_, frame_id_);
  // ... (省略其他基础参数加载) ...
  
  // 加载无人机物理参数
  pnh_.param("mass", mass_, mass_);
  pnh_.param("gravity", gravity_, gravity_);
  pnh_.param("inertia_xx", inertia_diag_[0], inertia_diag_[0]);
  pnh_.param("inertia_yy", inertia_diag_[1], inertia_diag_[1]);
  pnh_.param("inertia_zz", inertia_diag_[2], inertia_diag_[2]);

  // 加载位置环增益
  double kp_xy = pos_gains_.kp_xy;
  // ... (省略位置增益加载) ...
  pos_gains_.kp_xy = kp_xy;

  // 加载姿态环增益
  double kp_roll = att_gains_.kp_roll;
  // ... (省略姿态增益加载) ...
  att_gains_.kp_roll = kp_roll;
  
  // --- 电机布局参数加载 ---
  // 默认按照四旋翼 X 型布置初始化电机参数
  const double default_kappa = (motor_constant_ > 1e-9) ? (moment_constant_ / motor_constant_) : 0.0;
  const double diag = arm_length_ / std::sqrt(2.0); // 对角线长度的一半
  rotor_directions_ = {-1.0, 1.0, 1.0, -1.0};  // {前左, 前右, 后左, 后右} CW, CCW, CCW, CW
  rotor_xy_ = {Eigen::Vector2d(diag, diag), Eigen::Vector2d(diag, -diag),
               Eigen::Vector2d(-diag, diag), Eigen::Vector2d(-diag, -diag)};

  // 尝试从全局参数 `/uam_config/rotor_configuration` 加载更详细的电机布局
  // 这允许我们定义非对称或者更复杂的无人机结构
  ros::NodeHandle nh_global;
  XmlRpc::XmlRpcValue rotor_config;
  if (nh_global.getParam("/uam_config/rotor_configuration", rotor_config) &&
      rotor_config.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
      // ... (此处省略解析 XmlRpcValue 的详细代码) ...
      // 这部分代码会遍历配置中的每个电机，并更新其角度、力臂、转向等参数
  }
  
  // 同样，优先从全局配置加载惯量参数
  nh_global.param("/uam_config/inertia_quad_body/xx", inertia_diag_[0], inertia_diag_[0]);
  nh_global.param("/uam_config/inertia_quad_body/yy", inertia_diag_[1], inertia_diag_[1]);
  nh_global.param("/uam_config/inertia_quad_body/zz", inertia_diag_[2], inertia_diag_[2]);
}

/**
 * @brief 设置电机混合矩阵 (逆分配矩阵)
 *
 * 这个矩阵的作用是：输入期望的总推力 T 和三轴扭矩 τx, τy, τz，
 * 输出每个电机需要产生的推力 f1, f2, f3, f4。
 * [T, τx, τy, τz]^T = AllocationMatrix * [f1, f2, f3, f4]^T
 * 因此，我们需要求解 AllocationMatrix 的逆。
 */
void ControllerNode::setupMixer()
{
  Eigen::Matrix4d allocation;
  allocation.setZero();
  for (int i = 0; i < 4; ++i)
  {
    const double x = rotor_xy_[i].x();
    const double y = rotor_xy_[i].y();
    allocation(0, i) = 1.0;                           // 每个电机的推力都贡献到总推力
    allocation(1, i) = y;                             // 电机在y轴上的位置贡献到滚转力矩 (τx)
    allocation(2, i) = -x;                            // 电机在x轴上的位置贡献到俯仰力矩 (τy)
    allocation(3, i) = rotor_directions_[i] * rotor_kappa_[i]; // 旋转方向和力矩系数贡献到偏航力矩
  }

  // 检查矩阵是否奇异。如果接近奇异，使用伪逆以增加鲁棒性
  if (std::abs(allocation.determinant()) < 1e-6)
  {
    mixer_matrix_ = allocation.completeOrthogonalDecomposition().pseudoInverse();
    ROS_WARN("电机分配矩阵逼近奇异，已使用伪逆进行混合。");
  }
  else
  {
    mixer_matrix_ = allocation.inverse();
  }
}

/**
 * @brief 初始化所有ROS话题订阅者
 */
void ControllerNode::setupSubscriptions()
{
  odom_sub_ = nh_.subscribe(odom_topic_, 10, &ControllerNode::odomCallback, this);
  setpoint_sub_ = nh_.subscribe(setpoint_topic_, 10, &ControllerNode::setpointCallback, this);
  disturbance_sub_ = nh_.subscribe(disturbance_topic_, 10, &ControllerNode::disturbanceCallback, this);
}

/**
 * @brief 里程计消息回调函数
 */
void ControllerNode::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  // 更新无人机状态
  state_.stamp = msg->header.stamp;
  state_.position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  state_.velocity = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
  state_.angular_velocity = Eigen::Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
  tf2::fromMsg(msg->pose.pose.orientation, state_.orientation);

  // 对速度进行一阶低通滤波，以减少传感器噪声影响
  state_.filtered_velocity = velocity_filter_alpha_ * state_.velocity + (1.0 - velocity_filter_alpha_) * state_.filtered_velocity;
  state_.valid = true;

  UAM_DEBUG_DETAIL("里程计位置: " << state_.position.transpose());
  UAM_DEBUG_PUBLISH_VECTOR(log_pub_.get(), "/uam_controller/debug/position", state_.position);
}

/**
 * @brief 目标点消息回调函数
 */
void ControllerNode::setpointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  // 更新期望目标点
  setpoint_.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  setpoint_.stamp = msg->header.stamp;

  // 从四元数中提取期望偏航角
  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.orientation, q);
  if (std::abs(q.length2() - 1.0) < 1e-3) // 检查四元数是否有效
  {
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    setpoint_.yaw = yaw;
    setpoint_.has_yaw = true;
  }
  else
  {
    setpoint_.has_yaw = false;
  }
  setpoint_.valid = true;
  UAM_DEBUG_DETAIL("收到期望位置: " << setpoint_.position.transpose());
}

/**
 * @brief 扰动估计消息回调函数
 */
void ControllerNode::disturbanceCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
  disturbance_force_ = Eigen::Vector3d(msg->vector.x, msg->vector.y, msg->vector.z);
  disturbance_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  UAM_DEBUG_DETAIL("更新扰动估计: " << disturbance_force_.transpose());
}


/**
 * @brief 动态调参回调函数
 */
void ControllerNode::reconfigureCallback(uam_controller::uam_controllerConfig &cfg, uint32_t /*level*/)
{
  // 当在 rqt_reconfigure 中拖动滑块时，此函数被调用，实时更新参数
  enable_takeoff_ = cfg.enable_takeoff;
  pos_gains_.kp_xy = cfg.position_kp_xy;
  att_gains_.kp_roll = cfg.attitude_kp_roll;
  // ... (省略其他参数更新) ...

  // 参数更新后，重置积分项，避免突变
  resetIntegrators();
}

/**
 * @brief 主控制循环
 */
void ControllerNode::controlLoop(const ros::TimerEvent &event)
{
  if (!state_.valid) return; // 如果没有收到里程计数据，则不执行控制

  // --- 1. 前置检查与准备 ---
  if (!enable_takeoff_)
  {
    publishMotorCommand(Eigen::Vector4d::Zero()); // 未使能则输出零油门
    resetIntegrators();
    return;
  }
  
  const ros::Time now = ros::Time::now();
  double dt = 1.0 / control_rate_hz_; // 计算时间间隔 dt
  if (!last_control_time_.isZero()) {
    dt = std::max(1e-4, (now - last_control_time_).toSec());
  }
  last_control_time_ = now;

  // --- 2. 位置控制器 (外环) ---
  // 计算期望的加速度指令
  double desired_yaw = setpoint_.has_yaw ? setpoint_.yaw : target_yaw_;
  Eigen::Vector3d accel_cmd = computePositionControl(dt, desired_yaw);

  // --- 3. 计算总推力和期望姿态 ---
  // 扰动前馈补偿
  Eigen::Vector3d disturbance = Eigen::Vector3d::Zero();
  if (enable_feedforward_ && disturbanceFresh(now))
  {
    disturbance = disturbance_force_;
  }

  // 计算期望的合力：F_des = m*a_cmd + m*g - F_disturbance
  Eigen::Vector3d force_cmd = mass_ * accel_cmd;
  force_cmd += Eigen::Vector3d(0.0, 0.0, mass_ * gravity_); // 重力补偿
  force_cmd -= disturbance; // 扰动力补偿

  // 将合力向量转换为期望姿态 q_des 和总推力 thrust_cmd
  tf2::Quaternion q_des;
  double thrust_cmd = 0.0;
  computeOrientationTargets(force_cmd, desired_yaw, q_des, thrust_cmd);

  // --- 4. 姿态控制器 (内环) ---
  // 计算姿态误差
  tf2::Quaternion q_current = state_.orientation;
  tf2::Quaternion q_error = q_current.inverse() * q_des; 
  if (q_error.w() < 0.0) // 确保误差表示的是最短路径
  {
    q_error.setValue(-q_error.x(), -q_error.y(), -q_error.z(), -q_error.w());
  }
  
  // 将四元数误差近似为欧拉角误差向量
  tf2::Vector3 attitude_error_tf(2.0 * q_error.x(), 2.0 * q_error.y(), 2.0 * q_error.z());
  Eigen::Vector3d attitude_error_e(attitude_error_tf.x(), attitude_error_tf.y(), attitude_error_tf.z());

  // 更新积分项 (带抗饱和)
  attitude_integral_ += attitude_error_e * dt;
  for (int i = 0; i < 3; ++i)
  {
    attitude_integral_[i] = clamp(attitude_integral_[i], -integral_limit_, integral_limit_);
  }

  // PID 控制计算期望角加速度
  Eigen::Vector3d angular_rate = state_.angular_velocity;
  Eigen::Vector3d ang_acc_cmd;
  ang_acc_cmd.x() = att_gains_.kp_roll * attitude_error_e.x() + att_gains_.kd_roll * (-angular_rate.x()) + att_gains_.ki_roll * attitude_integral_.x();
  ang_acc_cmd.y() = att_gains_.kp_pitch * attitude_error_e.y() + att_gains_.kd_pitch * (-angular_rate.y()) + att_gains_.ki_pitch * attitude_integral_.y();
  double yaw_rate_error = use_yaw_rate_stabilizer_ ? (-angular_rate.z()) : 0.0;
  ang_acc_cmd.z() = att_gains_.kp_yaw * attitude_error_e.z() + att_gains_.kd_yaw * yaw_rate_error + att_gains_.ki_yaw * attitude_integral_.z();

  // 将期望角加速度转换为期望扭矩: τ = I * α
  Eigen::Vector3d torques = inertia_diag_.asDiagonal() * ang_acc_cmd;
  
  // --- 5. 电机混合与发布 ---
  // 将总推力和总扭矩混合为四个电机的推力
  Eigen::Vector4d rotor_forces = mixWrenchToThrust(thrust_cmd, torques);
  
  // 发布电机指令
  publishMotorCommand(rotor_forces);
}

/**
 * @brief 计算位置控制器的输出（期望加速度）
 */
Eigen::Vector3d ControllerNode::computePositionControl(double dt, double &desired_yaw)
{
  // 外环 P 控制：位置误差 -> 期望速度
  Eigen::Vector3d position_error = setpoint_.valid ? (setpoint_.position - state_.position) : Eigen::Vector3d::Zero();
  Eigen::Vector2d vel_cmd_xy = pos_gains_.kp_xy * position_error.head<2>();
  double vel_cmd_z = pos_gains_.kp_z * position_error.z();
  vel_cmd_z = clamp(vel_cmd_z, -max_descent_rate_, max_ascent_rate_); // 限制垂直速度

  // 内环 PD 控制：速度误差 -> 期望加速度
  Eigen::Vector2d vel_error_xy = vel_cmd_xy - state_.filtered_velocity.head<2>();
  double vel_error_z = vel_cmd_z - state_.filtered_velocity.z();

  Eigen::Vector3d accel_cmd;
  accel_cmd.x() = pos_gains_.kd_xy * vel_error_xy.x();
  accel_cmd.y() = pos_gains_.kd_xy * vel_error_xy.y();
  accel_cmd.z() = pos_gains_.kd_z * vel_error_z;

  // 限制最大水平加速度，间接限制了最大倾斜角
  double max_lateral_accel = gravity_ * std::tan(max_tilt_deg_ * kDegToRad);
  double accel_xy_norm = accel_cmd.head<2>().norm();
  if (accel_xy_norm > max_lateral_accel)
  {
    accel_cmd.head<2>() *= (max_lateral_accel / accel_xy_norm);
  }
  return accel_cmd;
}

/**
 * @brief 根据期望的合力向量计算期望姿态和总推力
 */
void ControllerNode::computeOrientationTargets(const Eigen::Vector3d &force_cmd,
                                                double yaw_des,
                                                tf2::Quaternion &q_des_out,
                                                double &thrust_out) const
{
  // 期望的机体 Z 轴方向应与期望的合力方向对齐
  Eigen::Vector3d zb_des = force_cmd.normalized();

  // 根据期望的偏航角，计算期望的机体 X 轴在水平面上的投影
  Eigen::Vector3d xc(std::cos(yaw_des), std::sin(yaw_des), 0.0);
  
  // 通过向量叉乘构建期望姿态的旋转矩阵 R_des = [xb_des, yb_des, zb_des]
  Eigen::Vector3d yb_des = zb_des.cross(xc).normalized();
  Eigen::Vector3d xb_des = yb_des.cross(zb_des);

  tf2::Matrix3x3 R_des(xb_des.x(), yb_des.x(), zb_des.x(),
                       xb_des.y(), yb_des.y(), zb_des.y(),
                       xb_des.z(), yb_des.z(), zb_des.z());

  // 从旋转矩阵中获取期望的四元数
  R_des.getRotation(q_des_out);
  q_des_out.normalize();

  // 总推力大小等于合力向量的模长
  thrust_out = clamp(force_cmd.norm(), 0.0, max_thrust_);
}

/**
 * @brief 将总推力和扭矩混合为四个电机的推力
 */
Eigen::Vector4d ControllerNode::mixWrenchToThrust(double thrust,
                                                   const Eigen::Vector3d &torques) const
{
  Eigen::Vector4d wrench;
  wrench << thrust, torques.x(), torques.y(), torques.z();
  
  // 使用之前计算好的混合矩阵（逆矩阵）进行计算
  Eigen::Vector4d rotor_forces = mixer_matrix_ * wrench;
  
  // 确保电机推力为正值
  for (int i = 0; i < 4; ++i)
  {
    if (!std::isfinite(rotor_forces[i]) || rotor_forces[i] < 0.0)
    {
      rotor_forces[i] = 0.0;
    }
  }
  return rotor_forces;
}

/**
 * @brief 将计算出的电机推力转换为油门指令并发布
 */
void ControllerNode::publishMotorCommand(const Eigen::Vector4d &forces)
{
  std_msgs::Float64MultiArray cmd;
  cmd.data.resize(4);
  for (int i = 0; i < 4; ++i)
  {
    // 将物理推力（牛顿）转换为归一化的油门值（0-1）
    double throttle = forces[i] / motor_force_scale_;
    throttle = clamp(throttle, motor_min_cmd_, motor_max_cmd_);
    if (!std::isfinite(throttle))
    {
      throttle = motor_min_cmd_;
    }
    cmd.data[i] = throttle;
    UAM_DEBUG_PUBLISH_SCALAR(log_pub_.get(), "/uam_controller/debug/throttle_" + std::to_string(i), throttle);
  }
  motor_pub_.publish(cmd);
}

/**
 * @brief 重置积分项
 */
void ControllerNode::resetIntegrators()
{
  attitude_integral_.setZero();
}

/**
 * @brief 检查扰动估计是否"新鲜"（未超时）
 */
bool ControllerNode::disturbanceFresh(const ros::Time &now) const
{
  if (disturbance_stamp_.isZero()) return false;
  return (now - disturbance_stamp_).toSec() < disturbance_timeout_;
}

/**
 * @brief 将数值限制在最小和最大值之间
 */
double ControllerNode::clamp(double value, double min_val, double max_val) const
{
  return std::max(min_val, std::min(max_val, value));
}

}  // 命名空间 uam_controller 结束

/**
 * @brief 主函数
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "uam_controller_node");
  uam_controller::ControllerNode node;
  node.spin();
  return 0;
}
