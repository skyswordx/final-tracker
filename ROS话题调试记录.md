# ROS 话题调试记录

## 话题连通性检查结果
- `/quad_ros` 正常发布 `/diy_odom`，`rostopic info /diy_odom` 显示发布者包含 `/quad_ros` 与 `/target_ekf_node`，订阅者包含 `/manager`（规划与建图 nodelet）和 `/uam_controller_node`。
- `rostopic echo -n1 /diy_odom` 可以获得实时里程计数据，说明 MuJoCo 仿真桥接工作正常。
- `/manager` nodelet 管理器已加载规划与建图节点，订阅 `/diy_odom`、`/target_ekf_odom`、`/diy_img/depth/image_raw`，并发布 `/gridmap_inflate`、`/trajectory` 等话题。
- `/gridmap_inflate` 当前只有发布者 `/manager`，执行 `timeout 5 rostopic echo -n1 /gridmap_inflate` 超时未收到数据，表明建图未产出有效栅格图。
- `rostopic info /diy_img/depth/image_raw` 显示 **无发布者**，只有 `/manager` 订阅，缺乏深度输入。

## 结论
- 规划模块出现 “no odom or no map” 的主要原因不是里程计缺失，而是建图模块未收到深度图像，因而无法生成栅格地图。

## 模块链路诊断
- **MuJoCo 仿真与控制器**：`rostopic info /quad/motor_cmd` 表明 `/uam_controller_node` 与 `/quad_ros` 正常互联，`rostopic echo -n1 /quad/motor_cmd` 可读到电机指令；同时，`rostopic info /diy_odom` 与 `rostopic echo -n1 /diy_odom` 证实仿真端持续发布里程计。
- **轨迹服务器与控制分配**：`rosnode info /traj_server` 显示其成功订阅 `/trajectory` 与 `/heartbeat` 并负责转发 `/uam_controller/cmd`。然而 `timeout 2 rostopic echo -n1 /uam_controller/cmd` 未收到消息，说明当前无有效轨迹生成。
- **规划节点链路**：`rostopic info /trajectory` 确认 `/manager`（nodelet 管理器）负责发布，但 `timeout 2 rostopic echo -n1 /trajectory` 超时未取到数据，进一步验证规划模块因缺图未下发轨迹。
- **建图模块**：`rosnode info /manager` 显示其订阅 `/diy_img/depth/image_raw` 并发布 `/gridmap_inflate`，但 `timeout 5 rostopic echo -n1 /gridmap_inflate` 没有输出，和深度缺失导致的建图空结果一致。
- **目标检测管线**：`rostopic info /diy_img/rgb/image_raw` 显示无发布者，`rosnode info /detect_node` 仅等待该话题输入；`rosnode info /d2p_node` 订阅 `/yolo_detector/detected_bounding_boxes` 及 `/iris_0/realsense/depth_camera/depth/image_raw`，但 `timeout 2 rostopic echo -n1 /target_3d_position` 爆超时，说明在未接入 UE AirSim 的 RGB/深度数据前，检测-三维定位链路无法产出结果。
- **目标 EKF**：`rostopic info /target_ekf_odom` 当前无发布者，因其依赖的 `/target_3d_position` 仍未更新。

## 建议的排查与验证步骤
1. 使用 `rostopic list`、`rostopic info` 或 `rosnode info` 核对关键话题与节点是否连接。
2. 通过 `rostopic echo -n1 /diy_odom`、`rostopic hz /quad/motor_cmd` 验证里程计与电机指令流是否正常。
3. 待提供深度流后，使用 `rostopic hz /gridmap_inflate` 或 `rostopic echo /gridmap_inflate` 确认建图输出是否恢复。
4. 如果暂时无法提供深度数据，可在 `mujoco_simulation.launch` 中临时关闭建图/规划依赖，或使用脚本发布伪造的 `/diy_img/depth/image_raw` 以便测试其余链路。

## 引入 UE AirSim 图像后的可行性评估
- 只要通过 AirSim 桥接发布 `/diy_img/rgb/image_raw` 与 `/diy_img/depth/image_raw`，现有 ROS 节点即能直接接入；`detect_node` 与 `/manager` 均已订阅对应话题。
- 一旦深度数据到位，`mapping` nodelet 能恢复输出 `/gridmap_inflate`，规划 nodelet 将重新生成 `/trajectory`，`traj_server` 随即向 `/uam_controller/cmd` 转发轨迹点，闭环控制链路即可正常驱动仿真。
- RGB 图像输入后，YOLO 检测与 `d2p_node` 将产出 `/target_3d_position`；`target_ekf_node` 随之发布 `/target_ekf_odom`，为规划提供目标状态，整套追踪链路闭合。
- 接入后建议的快速验证：
	1. `rostopic hz /diy_img/rgb/image_raw`、`rostopic hz /diy_img/depth/image_raw` 检查帧率。
	2. `rostopic hz /gridmap_inflate`、`rostopic hz /trajectory` 确认建图与规划产出。
	3. `rostopic echo -n1 /uam_controller/cmd`、`rostopic hz /quad/motor_cmd` 验证控制指令链路。
	4. `rostopic hz /target_ekf_odom` 检查目标追踪结果。
