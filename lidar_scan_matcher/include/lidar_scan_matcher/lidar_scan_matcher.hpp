#ifndef _LIDAR_SCAN_MATCHER_HPP_
#define _LIDAR_SCAN_MATCHER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pclomp/ndt_omp.h>

using PointType = pcl::PointXYZ;

class LidarScanMatcher : public rclcpp::Node
{
public:
  LidarScanMatcher(const rclcpp::NodeOptions & node_options);
  ~LidarScanMatcher() = default;

  void callback_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void callback_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);

  pcl::Registration<PointType, PointType>::Ptr get_registration();
  nav_msgs::msg::Odometry convert_to_odometry(const geometry_msgs::msg::Pose pose);

  pcl::PointCloud<PointType>::Ptr transform_point_cloud(
    const std::string source_frame, const std::string target_frame, const rclcpp::Time stamp,
    pcl::PointCloud<PointType>::Ptr input_cloud_ptr);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr scan_matcher_odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr scan_matcher_path_publisher_;

  // registration
  pcl::Registration<PointType, PointType>::Ptr registration_;

  geometry_msgs::msg::Pose current_pose_;

  std::deque<sensor_msgs::msg::Imu> imu_queue_;

  tf2_ros::Buffer tf_buffer_{ get_clock() };
  tf2_ros::TransformListener tf_listener_{ tf_buffer_ };
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

#endif
