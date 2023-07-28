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

#ifndef LIDAR_GRAPH_SLAM__LIDAR_SCAN_MATCHER_HPP_
#define LIDAR_GRAPH_SLAM__LIDAR_SCAN_MATCHER_HPP_

#include "lidar_graph_slam_utils/lidar_graph_slam_utils.hpp"

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lidar_graph_slam_msgs/msg/key_frame.hpp>
#include <lidar_graph_slam_msgs/msg/key_frame_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

using PointType = pcl::PointXYZI;

class LidarScanMatcher : public rclcpp::Node
{
public:
  LidarScanMatcher(const rclcpp::NodeOptions & node_options);
  ~LidarScanMatcher() = default;

  void callback_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void callback_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);

  void correct_imu(const sensor_msgs::msg::Imu imu_msg, const Eigen::Matrix4f & initial_guess);
  void get_latest_imu_msg(sensor_msgs::msg::Imu & latest_imu_msg);

  pcl::Registration<PointType, PointType>::Ptr get_registration();
  nav_msgs::msg::Odometry convert_to_odometry(const geometry_msgs::msg::Pose pose);

  geometry_msgs::msg::TransformStamped get_transform(
    const std::string source_frame, const std::string target_frame);
  pcl::PointCloud<PointType>::Ptr transform_point_cloud(
    const pcl::PointCloud<PointType>::Ptr input_cloud_ptr,
    const geometry_msgs::msg::TransformStamped transform);
  pcl::PointCloud<PointType>::Ptr transform_point_cloud(
    const pcl::PointCloud<PointType>::Ptr input_cloud_ptr, const Eigen::Matrix4f transform_matrix);

  void publish_tf(
    const geometry_msgs::msg::Pose pose, const rclcpp::Time stamp, const std::string frame_id,
    const std::string child_frame_id);
  void publish_key_frame(const lidar_graph_slam_msgs::msg::KeyFrame key_frame);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr front_end_map_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    scan_matcher_pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr scan_matcher_odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr scan_matcher_path_publisher_;
  rclcpp::Publisher<lidar_graph_slam_msgs::msg::KeyFrame>::SharedPtr key_frame_publisher_;

  pcl::PointCloud<PointType>::Ptr target_cloud_;

  // registration
  pcl::Registration<PointType, PointType>::Ptr registration_;

  std::string registration_type_;

  lidar_graph_slam_msgs::msg::KeyFrameArray key_frame_array_;

  Eigen::Matrix4f key_frame_;
  Eigen::Matrix4f translation_;
  Eigen::Matrix4f prev_translation_;

  geometry_msgs::msg::Pose current_pose_;

  nav_msgs::msg::Path estimated_path_;

  std::deque<sensor_msgs::msg::Imu> imu_queue_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  std::string base_frame_id_;
  std::string sensor_frame_id_;

  int max_scan_accumulate_num_;
  double displacement_;
  double accum_distance_{0.0};

  int id_{0};
};

#endif
