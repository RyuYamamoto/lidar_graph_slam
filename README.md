# LiDAR Graph SLAM

**InProgress**

## 1. Install
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recursive git@github.com:RyuYamamoto/lidar_graph_slam.git
cd ../
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 2. run
```bash
ros2 launch lidar_graph_slam lidar_graph_slam.launch.xml
```

[![](https://img.youtube.com/vi/Y_-EXSIKDWY/0.jpg)](https://www.youtube.com/watch?v=Y_-EXSIKDWY)

## ToDo
- [x] graph based SLAM(loop detect)
- [ ] save map
- [ ] improve loop detection
- [ ] use FPFH
- [ ] use original KD-Tree
