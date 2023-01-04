#ifndef _LIDAR_SCAN_MATCHER_HPP_
#define _LIDAR_SCAN_MATCHER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pclomp/ndt_omp.h>

class LidarScanMatcher : public rclcpp::Node
{
  using PointType = pcl::PointXYZ;

public:
  LidarScanMatcher();
  ~LidarScanMatcher() = default;

  void callback_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void callback_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);

  pcl::Registration<PointType, PointType>::Ptr get_registration();
  nav_msgs::msg::Odometry convert_to_odometry(const geometry_msgs::msg::Pose pose);

  pcl::PointCloud<PointType> transform_point_cloud(
    const std::string source_frame, const std::string target_frame,
    pcl::PointCloud<PointType>::Ptr cloud);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr scan_matcher_odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr scan_matcher_path_publisher_;

  // registration
  pcl::Registration<PointType, PointType>::Ptr registration_;

  geometry_msgs::msg::Pose current_pose_;

  std::deque<sensor_msgs::msg::Imu> imu_queue_;
};

#endif
