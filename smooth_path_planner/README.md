# 附加题：代价感知 A* 与路径平滑

适用环境为 Ubuntu 22.04 和 ROS 2 Humble。地图和 RViz 配置随包提供。

## 1. 检查 ROS 2 并安装依赖

在 Ubuntu 打开终端，确认 Humble 已安装：

```bash
source /opt/ros/humble/setup.bash
ros2 pkg list > /dev/null
```

如果找不到 setup.bash，请先安装 ROS 2 Humble Desktop，再继续。以下依赖只需安装一次：

```bash
sudo apt update
sudo apt install build-essential python3-colcon-common-extensions ros-humble-ament-cmake ros-humble-ament-index-python ros-humble-launch ros-humble-launch-ros ros-humble-rclcpp ros-humble-nav-msgs ros-humble-geometry-msgs ros-humble-rviz2 ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager ros-humble-nav2-costmap-2d ros-humble-tf2-ros unzip
```

## 2. 解压到工作空间

```bash
mkdir -p ~/ros_ws/src
unzip /mnt/hgfs/transferfiles/ros2-smooth-path.zip -d ~/ros_ws/src
ls ~/ros_ws/src/smooth_path_planner/package.xml
```

如果已有同名包，解压时会提示是否覆盖。先保留你自己的修改，再更新对应文件。

## 3. 编译附加题包

```bash
source /opt/ros/humble/setup.bash
cd ~/ros_ws
colcon build --packages-select smooth_path_planner --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

看到一个包编译成功后再启动。

## 4. 启动并在 RViz 中选点

先按 Ctrl+C 停止其他路径规划演示。在同一终端执行：

```bash
source ~/ros_ws/install/setup.bash
ros2 launch smooth_path_planner demo.launch.py
```

启动文件会自动配置并激活地图服务器与 Costmap，并打开 RViz。等待终端显示 Costmap ready，确认 RViz 中 Nav2 Costmap 的状态为 OK。

用 2D Pose Estimate 和 2D Goal Pose 选点。
灰线是平滑前的加权 A* 路线，绿线是三轮切角后的路线。每个切角的线段都检查代价地图的阻塞栅格，狭窄处可能保留折角。绿线是离散折线近似，没有为具体车型限制最小转弯半径。

移动成本为步长乘以 (1 + cost_weight × 栅格代价 / 100)。未知格和代价至少为 99 的格子禁止通行。增大 config/planner.yaml 中的 cost_weight 会加强远离墙壁的偏好。对比是否少贴墙时，用相同起终点对比基本版 A* 和此版。灰线和绿线用于对比平滑效果。

## 5. 下次运行

新终端中加载环境后直接启动。修改源码或配置后，按第 3 步重新编译。

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
ros2 launch smooth_path_planner demo.launch.py
```
