#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;
}

/**
 * @class SetpointPublisher
 * @brief 一个轻量级的参考轨迹发布节点。
 *
 * 该节点用于为控制器提供测试用的目标点。它可以生成多种
 * 简单的轨迹（如悬停、圆形、直线），并以固定的频率将
 * 期望的位姿发布到 ROS 话题上。
 */
class SetpointPublisher
{
public:
  SetpointPublisher()
  {
    loadParameters(); // 加载参数
    // 创建发布者，`latch` 设置为 true，可以确保后连接的订阅者能收到最后一条消息
    pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_, 10, true);
    // 创建定时器，用于周期性地调用 timerCallback
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &SetpointPublisher::timerCallback, this);
    start_time_ = ros::Time::now();
  }

  void spin()
  {
    ros::spin(); // 进入 ROS 事件循环
  }

private:
  /**
   * @brief 从参数服务器加载轨迹类型和相关参数
   */
  void loadParameters()
  {
    ros::NodeHandle pnh("~");
    pnh.param("publish_rate", publish_rate_, 20.0);
    pnh.param("start_delay", start_delay_, 1.0);
    pnh.param<std::string>("trajectory", trajectory_, std::string("hover"));
    pnh.param<std::string>("path_frame_id", frame_id_, std::string("world"));
    pnh.param<std::string>("setpoint_topic", topic_, std::string("/quad/position_setpoint"));

    // 加载悬停点坐标
    std::vector<double> hover_point_vec;
    if (pnh.getParam("hover_point", hover_point_vec) && hover_point_vec.size() == 3)
    {
      hover_point_ = Eigen::Vector3d(hover_point_vec[0], hover_point_vec[1], hover_point_vec[2]);
    }
    else
    {
      hover_point_ = Eigen::Vector3d(0.0, 0.0, 1.0);
    }
    
    // 加载圆形和直线轨迹的参数
    pnh.param("circle_radius", circle_radius_, 1.0);
    pnh.param("circle_period", circle_period_, 20.0);
    pnh.param("line_speed", line_speed_, 0.5);
    pnh.param("line_length", line_length_, 2.0);
  }

  /**
   * @brief 生成悬停轨迹的目标点
   */
  Eigen::Vector3d hoverTrajectory(double /*t*/) const
  {
    return hover_point_;
  }

  /**
   * @brief 生成圆形轨迹的目标点
   */
  Eigen::Vector3d circleTrajectory(double t) const
  {
    const double omega = kTwoPi / circle_period_; // 角速度
    double x = hover_point_.x() + circle_radius_ * std::cos(omega * t);
    double y = hover_point_.y() + circle_radius_ * std::sin(omega * t);
    double z = hover_point_.z();
    return Eigen::Vector3d(x, y, z);
  }

  /**
   * @brief 生成往复直线轨迹的目标点
   */
  Eigen::Vector3d lineTrajectory(double t) const
  {
    double half_length = 0.5 * line_length_;
    // 使用取模运算实现往复运动
    double distance = std::fmod(line_speed_ * t, 2.0 * line_length_);
    double offset = 0.0;
    if (distance <= line_length_)
    {
      offset = distance - half_length; // 从 -half_length 到 +half_length
    }
    else
    {
      offset = (2.0 * line_length_ - distance) - half_length; // 从 +half_length 回到 -half_length
    }
    double x = hover_point_.x() + offset;
    return Eigen::Vector3d(x, hover_point_.y(), hover_point_.z());
  }

  /**
   * @brief 定时器回调函数，用于计算并发布当前时刻的目标位姿
   */
  void timerCallback(const ros::TimerEvent &event)
  {
    const ros::Time now = ros::Time::now();
    if ((now - start_time_).toSec() < start_delay_)
    {
      return; // 如果未达到启动延迟时间，则不发布
    }

    const double t = (now - start_time_).toSec();
    Eigen::Vector3d pose;
    double yaw = 0.0;

    // 根据选择的轨迹类型计算目标点
    if (trajectory_ == "circle")
    {
      pose = circleTrajectory(t);
      // 计算轨迹切线方向作为期望偏航角，使无人机机头朝向飞行方向
      const double omega = kTwoPi / circle_period_;
      double dx_dt = -circle_radius_ * omega * std::sin(omega * t);
      double dy_dt = circle_radius_ * omega * std::cos(omega * t);
      yaw = std::atan2(dy_dt, dx_dt);
    }
    else if (trajectory_ == "line")
    {
      pose = lineTrajectory(t);
      yaw = 0.0; // 直线运动时，偏航角保持为0
    }
    else
    {
      pose = hoverTrajectory(t);
      yaw = 0.0; // 悬停时，偏航角保持为0
    }

    // 填充 PoseStamped 消息
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = now;
    msg.header.frame_id = frame_id_;
    msg.pose.position.x = pose.x();
    msg.pose.position.y = pose.y();
    msg.pose.position.z = pose.z();

    // 设置姿态（从RPY欧拉角转换为四元数）
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    q.normalize();
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();

    pub_.publish(msg);
  }

  // --- 成员变量 ---
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Timer timer_;

  double publish_rate_;      // 发布频率 (Hz)
  double start_delay_;       // 开始发布前的延迟 (s)
  std::string trajectory_;   // 轨迹类型 ("hover", "circle", "line")
  std::string frame_id_;     // 坐标系 ID
  std::string topic_;        // 发布的目标点话题名称
  Eigen::Vector3d hover_point_; // 悬停/轨迹中心点
  double circle_radius_;     // 圆形轨迹半径 (m)
  double circle_period_;     // 圆形轨迹周期 (s)
  double line_speed_;        // 直线运动速度 (m/s)
  double line_length_;       // 直线运动总长度 (m)
  ros::Time start_time_;     // 节点启动时间
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "setpoint_publisher");
  SetpointPublisher node;
  node.spin();
  return 0;
}
