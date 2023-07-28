// MIT License
//
// Copyright (c) 2023 Ryu Yamamoto
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef _LIDAR_GRAPH_SLAM_VIEWER_HPP_
#define _LIDAR_GRAPH_SLAM_VIEWER_HPP_

#include <Eigen/Dense>
#include <glk/effects/screen_space_lighting.hpp>
#include <glk/lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/pointcloud_buffer_pcl.hpp>
#include <glk/primitives/primitives.hpp>
#include <glk/thin_lines.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_interfaces/srv/set_rate.hpp>
#include <rosbag2_interfaces/srv/toggle_paused.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "lidar_graph_slam_msgs/msg/key_frame_array.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <fmt/core.h>
#include <implot.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <portable-file-dialogs.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

struct PlotBuffer
{
  ImVector<ImVec2> data;
  int max_queue_size;
  int offset;

  PlotBuffer() { max_queue_size = 2000; }
  void add(float x, float y)
  {
    if (data.size() < max_queue_size) {
      data.push_back(ImVec2(x, y));
    } else {
      data[offset] = ImVec2(x, y);
      offset = (offset + 1) % max_queue_size;
    }
  }
};

class LidarGraphSlamViewer : public rclcpp::Node
{
public:
  LidarGraphSlamViewer(const rclcpp::NodeOptions & node_options);
  ~LidarGraphSlamViewer();

private:
  void sensor_points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void key_frame_callback(const lidar_graph_slam_msgs::msg::KeyFrameArray::SharedPtr msg);
  void candidate_path_callback(const nav_msgs::msg::Path::SharedPtr msg);

  void create_viewer();
  void viewer_thread();

  geometry_msgs::msg::TransformStamped get_transform(
    const std::string source_frame, const std::string target_frame);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transform_point_cloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
    const geometry_msgs::msg::TransformStamped transform);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_subscriber_;
  rclcpp::Subscription<lidar_graph_slam_msgs::msg::KeyFrameArray>::SharedPtr
    key_frame_array_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr candidate_path_subscriber_;

  rclcpp::Client<rosbag2_interfaces::srv::TogglePaused>::SharedPtr toggle_client_;
  rclcpp::Client<rosbag2_interfaces::srv::SetRate>::SharedPtr rate_client_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  Eigen::Matrix4f latest_pose_matrix_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr latest_sensor_points_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_;

  nav_msgs::msg::Path candidate_path_;
  lidar_graph_slam_msgs::msg::KeyFrameArray key_frame_;
  std::vector<Eigen::Vector3f> key_frame_line_;

  std::thread viewer_thread_;

  float play_rate_{1.0f};

  bool is_candidate_key_frame_plot_{false};
  bool is_key_frame_plot_{false};
  bool is_map_loaded_{false};
  bool is_first_map_plot_{false};
};

#endif
