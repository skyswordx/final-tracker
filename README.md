# UAM Controller - Catkin工作空间设置指南

本文档记录了将UAM Controller包迁移到Catkin工作空间并成功编译的完整过程。

## 项目概述

UAM Controller是一个基于ROS的无人机控制器包，包含位置控制、速度控制和姿态控制功能。

## 目录结构

```
final-tracker/
├── catkin_ws/                    # Catkin工作空间
│   ├── src/                      # 源代码目录
│   │   └── uam_controller/       # UAM控制器包
│   ├── build/                    # 编译生成的中间文件
│   └── devel/                    # 开发空间（可执行文件和库）
└── README.md                     # 本文档
```

## 环境要求

- **操作系统**: Ubuntu 20.04
- **ROS版本**: ROS Noetic
- **编译器**: GCC 9.4.0+
- **依赖库**:
  - dynamic_reconfigure
  - geometry_msgs
  - nav_msgs
  - roscpp
  - std_msgs
  - tf2
  - tf2_geometry_msgs
  - tf2_ros
  - Eigen3

## 设置步骤

### 1. 创建Catkin工作空间

```bash
cd /home/circlemoon/final-tracker
mkdir -p catkin_ws/src
```

### 2. 移动软件包到src目录

```bash
mv /home/circlemoon/final-tracker/uam_controller /home/circlemoon/final-tracker/catkin_ws/src/
```

### 3. 初始化工作空间

```bash
cd /home/circlemoon/final-tracker/catkin_ws/src
catkin_init_workspace
```

这会创建一个指向ROS顶层CMakeLists.txt的符号链接。

### 4. 修复编译问题

在编译过程中，需要修复以下问题：

#### 4.1 移除不存在的Python脚本引用

编辑 `catkin_ws/src/uam_controller/CMakeLists.txt`，移除以下不存在的文件：
- `scripts/rls_adapter.py`
- `${PROJECT_SOURCE_DIR}/../neural-fly/ros_nodes/data_collector.py`

修改后的`catkin_install_python`部分：
```cmake
catkin_install_python(PROGRAMS
  scripts/control_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

#### 4.2 修复Eigen类型不匹配问题

在 `src/controller_node.cpp` 第319行，将三元运算符改为if-else语句：

修改前：
```cpp
Eigen::Vector3d position_error = setpoint_.valid ? (setpoint_.position - state_.position) : Eigen::Vector3d::Zero();
```

修改后：
```cpp
Eigen::Vector3d position_error;
if (setpoint_.valid) {
  position_error = setpoint_.position - state_.position;
} else {
  position_error = Eigen::Vector3d::Zero();
}
```

### 5. 编译工作空间

```bash
cd /home/circlemoon/final-tracker/catkin_ws
catkin_make
```

编译成功后，会在以下目录生成文件：
- `build/`: 编译中间文件
- `devel/`: 开发空间，包含可执行文件和库

## 使用方法

### 设置环境变量

每次使用前需要source工作空间：

```bash
source /home/circlemoon/final-tracker/catkin_ws/devel/setup.bash
```

建议将此命令添加到 `~/.bashrc` 文件中以自动加载：

```bash
echo "source /home/circlemoon/final-tracker/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 启动控制器节点

```bash
roslaunch uam_controller uam_controller.launch
```

### 运行独立节点

```bash
# 运行控制器节点
rosrun uam_controller controller_node

# 运行设定点发布器
rosrun uam_controller setpoint_publisher

# 运行控制监视器
rosrun uam_controller control_monitor.py
```

## 软件包内容

### 节点

1. **controller_node**: 主控制器节点，实现位置、速度和姿态控制
2. **setpoint_publisher**: 设定点发布器节点
3. **control_monitor.py**: 控制监视器脚本

### 配置文件

- `config/controller_params.yaml`: 控制器参数配置
- `config/uam_config.yaml`: UAM配置
- `cfg/ControllerConfig.cfg`: 动态重配置文件

### 启动文件

- `launch/uam_controller.launch`: 主启动文件

## 常见问题

### Q1: 编译时找不到依赖包

**解决方案**: 确保已安装所有ROS依赖：

```bash
sudo apt-get update
sudo apt-get install ros-noetic-geometry-msgs ros-noetic-nav-msgs \
                     ros-noetic-tf2 ros-noetic-tf2-geometry-msgs \
                     ros-noetic-tf2-ros ros-noetic-dynamic-reconfigure \
                     libeigen3-dev
```

### Q2: 找不到可执行文件

**解决方案**: 确保已source工作空间：

```bash
source /home/circlemoon/final-tracker/catkin_ws/devel/setup.bash
```

### Q3: 重新编译

如果需要清理并重新编译：

```bash
cd /home/circlemoon/final-tracker/catkin_ws
catkin_make clean
catkin_make
```

## 维护与开发

### 添加新的依赖

1. 在 `package.xml` 中添加依赖
2. 在 `CMakeLists.txt` 中的 `find_package()` 添加包名
3. 重新编译

### 修改代码后重新编译

```bash
cd /home/circlemoon/final-tracker/catkin_ws
catkin_make
```

如果只修改了Python脚本，无需重新编译，直接运行即可。

## 版本信息

- **创建日期**: 2025-10-14
- **ROS版本**: Noetic
- **工作空间**: catkin_make

## 联系信息

如有问题，请参考ROS官方文档：
- ROS Wiki: http://wiki.ros.org/
- Catkin教程: http://wiki.ros.org/catkin/Tutorials
