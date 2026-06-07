# LiDAR Graph SLAM

**InProgress**

https://github.com/user-attachments/assets/e2a65a66-5f91-484e-8aab-1fc81b32b6aa

## System OverView
![overview](doc/system_overview.png)

## 1. Install

### Build lidar_graph_slam

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
ros2 launch lidar_graph_slam lidar_graph_slam.launch.py
```
## 3. save map
The map is saved via an action. `resolution` is the voxel leaf size [m] used to
down-sample the dense map before saving (set `0.0` to save the dense map as-is).
```bash
ros2 action send_goal /save_map lidar_graph_slam_msgs/action/SaveMap "{resolution: 0.2, path: <MAP PATH>}"
```

## ToDo
- [x] graph based SLAM(loop detect)
- [x] save map
- [ ] improve loop detection
- [ ] implement other lidar odometry algorithm
- [ ] use FPFH
- [ ] use original KD-Tree
