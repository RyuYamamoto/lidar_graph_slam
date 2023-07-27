# LiDAR Graph SLAM

**InProgress**

## 1. Install
### 1.1 Build Iridescence
See [Iridescence](https://github.com/koide3/iridescence) and try to install.

### 1.2 Build lidar_graph_slam

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recursive git@github.com:RyuYamamoto/lidar_graph_slam.git
cd ../
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 2. run
```bash
ros2 launch lidar_graph_slam lidar_graph_slam.launch.xml
```

[![](https://img.youtube.com/vi/hhWxuyCu7Us/0.jpg)](https://www.youtube.com/watch?v=hhWxuyCu7Us)

## ToDo
- [x] graph based SLAM(loop detect)
- [ ] save map
- [ ] improve loop detection
- [ ] use FPFH
- [ ] use original KD-Tree
