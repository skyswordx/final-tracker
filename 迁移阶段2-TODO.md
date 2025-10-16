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