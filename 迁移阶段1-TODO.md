
# Fast-Tracker 适配 Mujoco 与 uam_controller 工作计划（基于现状勘查）

## 1. 目标与现状差异

目标依旧是：用 MuJoCo 仿真器（`sim/quad_ros.py`）替换 Gazebo，使用 `uam_controller` 控制并完成感知-规划-控制闭环。本阶段先统一各 ROS 话题，使系统在没有本地 RGB/深度相机的情况下也能正常运行，并为未来对接 UE AirSim（提供 `/diy_img/rgb/image_raw`、`/diy_img/depth/image_raw` 等话题）留好接口。

实际仓库与旧 TODO 的不符点：
- `sim/quad_ros.py` 目前仅发布 `/quad/odometry`，订阅 `/quad/motor_cmd`，尚未输出相机话题，也不在任何 catkin 包中，无法通过 `<node pkg="..." type="..." />` 直接启动。
- `d2p` 模块的图像话题在脚本中硬编码为 `/iris_0/realsense/depth_camera/color/image_raw`，`launch` 文件没有 `image_topic` 参数。
- `target_ekf/launch/target_ekf.launch` 启动的是 `target_ekf_node`，只 remap 到 `/target_3d_position` 与 `/target_ekf_odom`，并没有深度图输入；`target_ekf_sim_node` 也未被使用。
- `planning/planning/launch/planning.launch` 没有位置指令话题参数，`traj_server` 节点是单独的可执行文件而非 nodelet。
- `uam_controller` 默认的话题为 `/quad/*`，`launch` 中没有提供自定义话题的 `arg`。

因此需要先按现有代码重新规划改造步骤。

## 2. 改造任务清单

### 2.1 MuJoCo 仿真节点改造
- **目标：** 让 `quad_ros.py` 成为独立的 ROS 节点，并发布/订阅统一的里程计、控制话题；同时预留相机话题命名，以便未来 AirSim 接入即可使用。
- **动作：**
  - 新建 catkin 包（建议命名 `py_sim`）并把 `sim/quad_ros.py` 安装为可执行文件；或者在现有包内添加 `CMakeLists.txt` / `package.xml` 以便 `roslaunch`。
  - 在脚本中读取私有参数：`~odom_topic`（默认 `/diy_odom`）、`~motor_topic`（默认 `/quad/motor_cmd` 或 `/uam_controller/motor_cmd`）等，并在 `roslaunch` 中配置。
  - 将控制输入改为监听统一的推力指令或 SO(3) 指令。若 `uam_controller` 输出 SO3，需要在仿真节点内实现 SO3 → 电机推力的转换或新增中间节点。
  - 预留 `~rgb_topic`、`~depth_topic`、`~camera_info_topic` 参数但不实际发布，确保未来 AirSim 发布对应话题时能够直接 remap。

### 2.2 感知链路
- **d2p**
  - 在 `scripts/yolo_detector/yolo_detector.py` 中将 `img_topic` 改为 ROS 参数，默认值 `/diy_img/rgb/image_raw`。若需要兼容旧话题，可把旧值设为参数默认值，通过顶层 launch remap。
  - 更新 `launch/d2p.launch`，为 `yolo_detector_node.py` 设置 `image_topic` 私有参数。改造完成后，只要 AirSim 发布 `/diy_img/rgb/image_raw`，该模块即可直接接入。
- **target_ekf**
  - 将 `launch/target_ekf.launch` 调整为启动 `target_ekf_sim_node`（该节点原生支持 `~/odom` 与 `~/depth`）。
  - 新增 `<arg>` 或 `<remap>` 使其订阅 `/diy_odom`、`/diy_img/depth/image_raw`、YOLO 输出。当前无深度话题时节点会等待数据，不会影响整体运行；当 AirSim 发布后，无需再次修改即可使用。
  - 若必须保留 `target_ekf_node`，则在 C++ 中同步增加深度订阅并统一接口。

### 2.3 建图
- `mapping/launch/mapping.launch` 已支持 `odom_topic`、`depth_topic` 参数，只需把默认值改为 `/diy_odom` 与 `/diy_img/depth/image_raw`，并在顶层 launch 里显式传入。当前阶段可通过参数保留旧话题以兼容调试，未来 AirSim 接入后直接切换即可。

### 2.4 规划模块
- 在 `planning/planning/launch/planning.launch` 顶部添加：
  - `arg name="odom_topic_" default="/diy_odom"`（已有默认值需更新）。
  - `arg name="position_cmd_topic" default="/position_cmd"`。
- 更新 `planning` nodelet 的 `<remap from="~odom" ...>` 使用新参数。
- 更新 `traj_server` 节点的 `<remap from="~position_cmd" ...>`，使用 `$(arg position_cmd_topic)`。

### 2.5 控制器
- 在 `uam_controller/launch/uam_controller.launch` 中：
  - 增加 `<arg>`：`odom_topic`、`setpoint_topic`、`motor_topic`、`cmd_topic`，默认与旧话题保持一致；如后续需要，可再新增 `so3_cmd_topic`。
  - 给 `controller_node` 设置 `~odom_topic`、`~motor_cmd_topic` 等私有参数，以便对接统一话题。
  - 给 `setpoint_publisher` 传递 `~setpoint_topic`，或在新的架构下考虑关闭该节点，改为接收规划结果。
  - 如需 SO3 指令输出，在 `controller_node` 中增加相应发布，并在顶层 launch 中配置 remap 或中间转换。

### 2.6 顶层 launch（新建）
- 新建 `catkin_ws/src/tracker/planning/planning/launch/mujoco_simulation.launch`，职责：
  1. 启动 MuJoCo 节点（来自上文新建包），设置帧频和话题，并通过参数统一发布/订阅名称。
  2. 启动 `d2p`、`mapping`、`target_ekf`，并传入统一话题参数（即使当前没有图像与深度数据，也先保持一致）。
  3. 启动 `planning`，将 `position_cmd_topic` remap 至 `/uam_controller/cmd`。
  4. 启动 `uam_controller`，配置其输入为 `/diy_odom`，输出电机推力或 SO3 指令供仿真节点使用。
  5. 按需启动 RViz。
- TODO：在该 launch 中理清话题映射，确保 `planning → uam_controller → quad_ros.py` 闭环。

## 3. 验证与后续

1. **统一话题自检**：`catkin_make` 后运行 `roslaunch planning mujoco_simulation.launch`，确认：
  - `quad_ros.py`（或仿真节点）已按默认参数发布 `/diy_odom`，并能接受 `uam_controller` 的控制指令。
  - TODO 中配置的 remap 生效，`planning`、`mapping`、`target_ekf` 等节点都订阅到统一话题（即使没有图像或深度数据，也不会异常退出）。
2. **模拟相机话题测试**：在未接入 AirSim 的情况下，用 `rostopic pub` 或脚本向 `/diy_img/rgb/image_raw`、`/diy_img/depth/image_raw` 发布示例消息，验证 `d2p`、`target_ekf`、`mapping` 能够正常处理并驱动规划输出。
3. **AirSim 接入准备**：未来当 AirSim 按照约定话题发布 RGB 与深度数据时，仅需启动 AirSim → ROS 桥，无需修改本仓库即可形成完整闭环。必要时更新相机内参配置。
4. **后续优化**：
  - 根据联调情况决定是否长期使用电机推力接口或切换到 SO3 指令。
  - 结合 AirSim 实际帧率与延迟，调节 `uam_controller` 与规划参数。

> 注：以上步骤切合当前仓库结构，实施前请逐项创建 Git 任务或 issue 以便跟踪。
