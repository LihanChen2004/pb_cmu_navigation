# pb_cmu_navigation

深圳北理莫斯科大学 北极熊战队 2025赛季哨兵导航仿真/实车包

![spin_nav.gif](https://cdn.jsdelivr.net/gh/LihanChen2004/picx-images-hosting/spin_nav.1ove3nw63o.gif)

## 一. 项目介绍

本项目基于 [CMU 导航框架](https://github.com/HongbiaoZ/autonomous_exploration_development_environment/tree/humble) 开发，主要修改内容如下：

1. 修改 local_planner，由差速控制改为全向控制。

2. 完整化 TF Tree，实现整车仿真建模。

3. 将原有 frame_id `map` 修改为 `odom`，以便后续加入重定位模块。

4. 在 sensor_scan_generation 中接收 LIO 输出的 lidar_odometry，转换并发布 `tf` : `odom -> chassis` 和 `Odometry` : `odom -> gimbal_yaw`。因为速度参考系基于 `gimbal_yaw` 。

5. 抛弃了 vehicle_simulator 中的节点，改为接收 Ignition Fortress 的 Pointcloud2 消息，再经过 ign_sim_pointcloud_tool 处理添加每个 point 的时间戳，再交由 LIO 输出 lidar_odometry。更加贴近实车情况。

## 二. 环境配置

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Ignition: [Fortress](https://gazebosim.org/docs/fortress/install_ubuntu/)

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

```zsh
ros2 launch cmu_nav_bringup bringup_sim.launch.py
```
