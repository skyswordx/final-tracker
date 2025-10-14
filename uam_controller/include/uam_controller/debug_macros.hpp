#pragma once

#include <ros/ros.h>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>

// --- 调试等级定义 ---
// UAM_DEBUG_LEVEL_NONE:   不输出任何调试信息
// UAM_DEBUG_LEVEL_FLOW:   输出关键流程信息（如初始化、模式切换）
// UAM_DEBUG_LEVEL_DETAIL: 输出高频的详细控制数据（如误差、指令）
#define UAM_DEBUG_LEVEL_NONE 0
#define UAM_DEBUG_LEVEL_FLOW 1
#define UAM_DEBUG_LEVEL_DETAIL 2

// 默认调试等级。可以在编译时通过 CMake 参数覆盖，例如:
// catkin build uam_controller --cmake-args -DUAM_DEBUG_LEVEL=2
#ifndef UAM_DEBUG_LEVEL
#define UAM_DEBUG_LEVEL UAM_DEBUG_LEVEL_DETAIL
#endif

// --- 调试信息限流配置 (单位: 秒) ---
// 用于防止高频调试信息刷屏
#ifndef UAM_DEBUG_FLOW_THROTTLE_SEC
#define UAM_DEBUG_FLOW_THROTTLE_SEC 1.0
#endif

#ifndef UAM_DEBUG_DETAIL_THROTTLE_SEC
#define UAM_DEBUG_DETAIL_THROTTLE_SEC 1.0
#endif


namespace uam_controller
{
namespace debug
{
// 内部辅助函数，用于根据文件名、行号和标签生成唯一的键
inline std::string makeKey(const char *file, int line, const char *tag)
{
  std::ostringstream oss;
  oss << tag << ':' << file << ':' << line;
  return oss.str();
}

// 内部辅助函数，判断是否应该记录日志（基于节流阀）
inline bool shouldLog(const std::string &key, double throttle_sec)
{
  static std::unordered_map<std::string, ros::Time> last_emit;
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock(mutex);
  const ros::Time now = ros::Time::now();
  ros::Time &stamp = last_emit[key];
  if (stamp.isZero() || (now - stamp).toSec() >= throttle_sec)
  {
    stamp = now;
    return true;
  }
  return false;
}
}  // namespace debug
}  // namespace uam_controller

// --- 宏定义 ---

// UAM_DEBUG_FLOW: 用于打印流程控制信息的宏
#if UAM_DEBUG_LEVEL >= UAM_DEBUG_LEVEL_FLOW
#define UAM_DEBUG_FLOW(stream_expr)                                                                 \
  do                                                                                                \
  {                                                                                                 \
    const std::string _key = ::uam_controller::debug::makeKey(__FILE__, __LINE__, "FLOW");          \
    if (::uam_controller::debug::shouldLog(_key, UAM_DEBUG_FLOW_THROTTLE_SEC))                      \
    {                                                                                               \
      std::ostringstream _oss;                                                                      \
      _oss << "[FLOW] " << stream_expr;                                                             \
      std::cout << _oss.str() << std::endl;                                                         \
    }                                                                                               \
  } while (0)
#else
#define UAM_DEBUG_FLOW(stream_expr) do {} while (0) // 如果调试等级不够，宏展开为空
#endif

// UAM_DEBUG_DETAIL: 用于打印详细调试数据的宏
#if UAM_DEBUG_LEVEL >= UAM_DEBUG_LEVEL_DETAIL
#define UAM_DEBUG_DETAIL(stream_expr)                                                               \
  do                                                                                                \
  {                                                                                                 \
    const std::string _key = ::uam_controller::debug::makeKey(__FILE__, __LINE__, "DETAIL");        \
    if (::uam_controller::debug::shouldLog(_key, UAM_DEBUG_DETAIL_THROTTLE_SEC))                    \
    {                                                                                               \
      std::ostringstream _oss;                                                                      \
      _oss << "[DETAIL] " << stream_expr;                                                           \
      std::cout << _oss.str() << std::endl;                                                         \
    }                                                                                               \
  } while (0)
#else
#define UAM_DEBUG_DETAIL(stream_expr) do {} while (0) // 如果调试等级不够，宏展开为空
#endif

// UAM_DEBUG_PUBLISH_*: 用于通过 LogPublisher 发布调试话题的宏
#if UAM_DEBUG_LEVEL >= UAM_DEBUG_LEVEL_DETAIL
#define UAM_DEBUG_PUBLISH_SCALAR(logger_ptr, topic, value)                           \
  do                                                                                 \
  {                                                                                  \
    if ((logger_ptr) != nullptr)                                                     \
    {                                                                                \
      (logger_ptr)->publishScalar(topic, value);                                     \
    }                                                                                \
  } while (0)

#define UAM_DEBUG_PUBLISH_VECTOR(logger_ptr, topic, vec)                             \
  do                                                                                 \
  {                                                                                  \
    if ((logger_ptr) != nullptr)                                                     \
    {                                                                                \
      (logger_ptr)->publishVector(topic, vec);                                       \
    }                                                                                \
  } while (0)
#else
#define UAM_DEBUG_PUBLISH_SCALAR(logger_ptr, topic, value) do {} while (0) // 如果调试等级不够，宏展开为空
#define UAM_DEBUG_PUBLISH_VECTOR(logger_ptr, topic, vec) do {} while (0)
#endif
