# ndt_slam

## How to use
```
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/RyuYamamoto/ndt_slam.git
cd ~/ros2_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --catkin-skip-building-tests
ros2 launch ndt_slam ndt_slam.launch.xml
ros2 launch ndt_slam tf_world_to_map.launch.xml
```
open other terminal,
```
ros2 bag play <ROSBAG PATH>
```

## Required packages
- [ndt_omp](https://github.com/koide3/ndt_omp)
- [ndt_slam_srvs](https://github.com/RyuYamamoto/ndt_slam_srvs)

## Video
[![](https://img.youtube.com/vi/ncyMT3vk7H4/0.jpg)](https://www.youtube.com/watch?v=ncyMT3vk7H4)
