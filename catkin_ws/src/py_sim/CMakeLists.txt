cmake_minimum_required(VERSION 3.0.2)
project(py_sim)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  geometry_msgs
  std_msgs
  quadrotor_msgs
)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/quad_ros.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY scene/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scene
)
