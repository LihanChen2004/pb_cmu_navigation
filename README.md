# pb_cmu_navigation

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![Build](https://github.com/LihanChen2004/pb_cmu_navigation/actions/workflows/ci.yml/badge.svg?branch=humble)](https://github.com/LihanChen2004/pb_cmu_navigation/actions/workflows/ci.yml)
[![Codacy grade](https://img.shields.io/codacy/grade/1a5495d4fddf48e4baede6e2351d7d7d)](https://app.codacy.com/gh/LihanChen2004/pb_cmu_navigation/dashboard?utm_source=gh&utm_medium=referral&utm_content=&utm_campaign=Badge_grade)

> **仍在开发中，更新频率较快且不稳定，不考虑向前兼容。请谨慎使用**

深圳北理莫斯科大学 北极熊战队 2025赛季哨兵导航仿真/实车包

| rmul_2024 小陀螺 | rmuc_2024 赛博飞坡 + 先验 pcd 里程计 |
|:-----------------:|:--------------:|
|![spin_nav.gif](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/spin_nav.1ove3nw63o.gif)|![rmuc_fly.gif](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/rmuc_fly_image.1aoyoashvj.gif)|

| 使用 NAV2 输出全局路径 |
|:-------------:|
|![nav_with_global_path](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/nav_with_global_path.6ik9ck07h1.gif)|

## 一. 项目介绍

本项目基于 [CMU 导航框架](https://github.com/HongbiaoZ/autonomous_exploration_development_environment/tree/humble) 开发，主要修改内容如下：

1. 修改 local_planner，由差速控制改为全向控制。

2. 完整化 TF Tree，实现整车仿真建模。

3. 将原有 frame_id `map` 修改为 `odom`，以便后续加入重定位模块。

4. 在 sensor_scan_generation 中接收 LIO 输出的 lidar_odometry，转换并发布 `tf` : `odom -> chassis` 和 `Odometry` : `odom -> gimbal_yaw`。因为速度参考系基于 `gimbal_yaw` 。

5. 抛弃了 vehicle_simulator 中的节点，改为接收 Ignition Fortress 的 Pointcloud2 消息，再经过 ign_sim_pointcloud_tool 处理添加每个 point 的时间戳，再交由 LIO 输出 lidar_odometry。更加贴近实车情况。

6. 使用 NAV2 的 Global Planner 作为全局路径规划器，再从 Global Plan 中裁剪出目标点，交由 CMU 的 Local Planner 进行局部路径规划。

## 二. 环境配置

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Ignition: [Fortress](https://gazebosim.org/docs/fortress/install_ubuntu/)
- 配套仿真包：[rm_gazebo_simulator](https://github.com/LihanChen2004/rmul24_gazebo_simulator)

1. 安装 [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)

    ```sh
    sudo apt install cmake
    ```

    ```sh
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    cd ./Livox-SDK2/
    mkdir build
    cd build
    cmake .. && make -j
    sudo make install
    ```

2. 克隆仓库

    ```zsh
    mkdir -p ~/ros_ws/src
    cd ~/ros_ws/src
    ```

    ```zsh
    git clone --recursive https://github.com/LihanChen2004/pb_cmu_navigation.git
    ```

3. 安装依赖

    ```zsh
    cd ~/ros_ws
    rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

4. 编译

    ```zsh
    colcon build --symlink-install -DCMAKE_BUILD_TYPE=Release
    ```

## 三. 运行

- 可选参数

  - `world` : 仿真世界名，关联栅格地图的读取。可选参数 `rmul_2024` or `rmuc_2024`。。

- RVIZ 插件

  `Way Point`: 规划局部路径

  `2D Goal Pose`: 规划全局路径

  `Goal Point`: Far Planner 的全局路径规划（即将弃用）

- 单机器人

    ```zsh
    ros2 launch cmu_nav_bringup rm_sentry_simulation_launch.py \
    world:=rmul_2024
    ```

- 多机器人

    当前指定的初始位姿实际上是无效的（）

    TODO: 加入 `map` -> `odom` 的变换和初始化

    ```zsh
    ros2 launch cmu_nav_bringup rm_multi_sentry_simulation_launch.py \
    world:=rmul_2024 \
    robots:=" \
    red_standard_robot1={x: 0.0, y: 0.0, yaw: 0.0}; \
    blue_standard_robot1={x: 5.6, y: 1.4, yaw: 3.14}; \
    "
    ```

    ```zsh
    ros2 launch cmu_nav_bringup rm_multi_sentry_simulation_launch.py \
    world:=rmuc_2024 \
    robots:=" \
    red_standard_robot1={x: 0.0, y: 0.0, yaw: 0.0}; \
    blue_standard_robot1={x: 0.0, y: 0.0, yaw: 0.0}; \
    "
    ```
