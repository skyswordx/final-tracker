我现在需要参考或者修改我的 airsim_ros_bridge.py 文件，以便支持
- 利用 airsim api 相关的接口，从 ue5 airsim 环境中获取深度图像信息，RGB图像信息，然后发布到对应的话题
- 把 mujoco 中，无人机的位置姿态信息等，发布给 ue5 airsim 环境进行渲染
- 删除这个文件中，有关机械臂、关节、夹爪相关的接口
- 合理的结合 py_sim 中的现成仿真包

相关 airsim api 文件查看
```shell
airsim_bridge/airsim
airsim_bridge/multirotor
```
主要接口基本上在 airsim_ros_bridge.py 文件已经调用了


## mujoco 和 ekf 的话题冲突

我发现airsim中出现了飞机模型渲染的瞬间移动到人物周围的情况，但是大部分时间还是留在了原地，哪怕d2p模块输出了3d点给tracker模块，tracker模块还是没办法正常工作，我现在怀疑是mujoco和tracker、airsim ros py这三个关于飞机位置姿态相关的话题数据或者覆盖存在连接失败或者冲突


- 之前 target_ekf.launch 同时存在 `<remap from="~odom" to="/diy_odom">`，导致目标 EKF 与 MuJoCo 仿真都在 `/diy_odom` 上发布 `nav_msgs/Odometry`。MuJoCo 控制器与 AirSim 桥接都订阅该话题，因此一旦目标 EKF 发布了新的目标位置，飞机里程计被覆盖，AirSim 中的机体会瞬移到目标附近，而控制器也以为无人机已到位，从而不再输出运动指令。  
- 我已经删除了这条问题 remap，并保留 `<remap from="target_ekf_node/odom" to="$(arg target_topic)">`，使目标 EKF 仅在 `/target_ekf_odom`（默认值）上发布。你之后只需重新编译并重新启动 Launch：  
  1. `catkin_make -C catkin_ws`  
  2. `source catkin_ws/devel/setup.bash`  
  3. 重新启动 `roslaunch planning mujoco_simulation.launch`（或等效组合）。  
- 修改后，用 `rostopic info /diy_odom` 验证发布者只剩 `/quad_ros`；再用 `rostopic echo -n1 /target_ekf_odom` 确认 EKF 输出转移到了独立话题。此时 AirSim 应与 MuJoCo 姿态保持一致，planner 也能得到正确的目标状态。

已进行更改。