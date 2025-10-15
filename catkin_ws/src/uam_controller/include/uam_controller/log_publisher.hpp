#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>

#include <Eigen/Core>
#include <iostream>
#include <string>
#include <unordered_map>

namespace uam_controller
{

/**
 * @class LogPublisher
 * @brief 一个按需创建和发布调试话题的辅助类。
 *
 * 该类的主要目的是简化调试过程。你不需要在代码中预先声明所有的
 * `ros::Publisher`。相反，你只需要调用 `publishScalar` 或 `publishVector`，
 * 这个类会自动检查是否已经为该话题创建了发布者。如果没有，它会
 * 动态地创建一个，然后发布消息。这使得在控制器代码中临时添加
 * 调试输出变得非常方便，便于在 rqt_plot 等工具中可视化内部变量。
 */
class LogPublisher
{
public:
  /**
   * @brief 构造函数
   * @param nh ROS 节点句柄
   */
  explicit LogPublisher(ros::NodeHandle &nh) : nh_(nh) {}

  /**
   * @brief 发布一个标量（double）数据
   * @param topic_name 话题名称
   * @param data 要发布的数据
   */
  void publishScalar(const std::string &topic_name, double data)
  {
    // 查找是否已存在该话题的发布者
    auto it = scalar_publishers_.find(topic_name);
    if (it == scalar_publishers_.end())
    {
      // 如果不存在，则创建一个新的发布者并存储起来
      ros::Publisher pub = nh_.advertise<std_msgs::Float64>(topic_name, 10);
      scalar_publishers_.emplace(topic_name, pub);
      std::cout << "创建标量调试话题: " << topic_name << std::endl;
    }

    // 发布消息
    std_msgs::Float64 msg;
    msg.data = data;
    scalar_publishers_[topic_name].publish(msg);
  }

  /**
   * @brief 发布一个三维向量 (Eigen::Vector3d) 数据
   * @param topic_name 话题名称
   * @param vec 要发布的向量
   */
  void publishVector(const std::string &topic_name, const Eigen::Vector3d &vec)
  {
    publishVector(topic_name, vec.x(), vec.y(), vec.z());
  }

  /**
   * @brief 发布一个三维向量 (x, y, z) 数据
   * @param topic_name 话题名称
   * @param x X 分量
   * @param y Y 分量
   * @param z Z 分量
   */
  void publishVector(const std::string &topic_name, double x, double y, double z)
  {
    // 逻辑同 publishScalar
    auto it = vector_publishers_.find(topic_name);
    if (it == vector_publishers_.end())
    {
      ros::Publisher pub = nh_.advertise<geometry_msgs::Vector3>(topic_name, 10);
      vector_publishers_.emplace(topic_name, pub);
      std::cout << "创建向量调试话题: " << topic_name << std::endl;
    }

    geometry_msgs::Vector3 msg;
    msg.x = x;
    msg.y = y;
    msg.z = z;
    vector_publishers_[topic_name].publish(msg);
  }

private:
  ros::NodeHandle &nh_; // ROS 节点句柄的引用
  // 使用哈希表存储话题名称到发布者的映射，提高查找效率
  std::unordered_map<std::string, ros::Publisher> scalar_publishers_;
  std::unordered_map<std::string, ros::Publisher> vector_publishers_;
};

}  // 命名空间 uam_controller 结束
