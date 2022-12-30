#ifndef _NDT_MAPPING_
#define _NDT_MAPPING_

#include <ndt_slam_srvs/srv/save_map.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>

#include <ndt_slam/data_struct.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclomp/ndt_omp.h>

//#include <eigen_conversions/eigen_msg.h>
#include "tf2/transform_datatypes.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#ifdef USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

class NDTSlam : public rclcpp::Node
{
  using PointType = pcl::PointXYZI;

public:
  NDTSlam();
  ~NDTSlam() = default;

private:
  void limitCloudScanData(
    const pcl::PointCloud<PointType>::Ptr input_ptr,
    const pcl::PointCloud<PointType>::Ptr & output_ptr, const double min_scan_range,
    const double max_scan_range);
  void downsample(
    const pcl::PointCloud<PointType>::Ptr input_ptr,
    const pcl::PointCloud<PointType>::Ptr & output_ptr);

  Pose getCurrentPose();

  void pointsCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_points_ptr_msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr & msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr & msg);

  void imuCorrect(const rclcpp::Time current_scan_time);
  void imuCorrect(Eigen::Matrix4f & pose, const rclcpp::Time stamp);

  geometry_msgs::msg::TransformStamped getTransform(
    const std::string target_frame, const std::string source_frame);
  void transformPointCloud(
    pcl::PointCloud<PointType>::Ptr input_ptr, pcl::PointCloud<PointType>::Ptr & output_ptr,
    const std::string target_frame, const std::string source_frame);

  bool saveMapService(
    const ndt_slam_srvs::srv::SaveMap::Request::SharedPtr req,
    ndt_slam_srvs::srv::SaveMap::Response::SharedPtr res);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ndt_aligned_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ndt_map_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ndt_pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr transform_probability_publisher_;

  rclcpp::Service<ndt_slam_srvs::srv::SaveMap>::SharedPtr save_map_service_;

  Pose ndt_pose_;
  Pose previous_pose_;

  Eigen::Matrix4f pose_{Eigen::Matrix4f::Identity()};

  Eigen::Vector3f imu_rotate_vec_;

  rclcpp::Time previous_scan_time_;

  pcl::PointCloud<PointType>::Ptr map_;
  pclomp::NormalDistributionsTransform<PointType, PointType> ndt_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  bool use_imu_{false};
  std::string base_frame_id_;

  // rosparam
  double min_scan_range_;
  double max_scan_range_;
  double min_add_scan_shift_;

  // voxel grid filter
  double leaf_size_;

  // NDT config rosparam
  double trans_eps_;
  double step_size_;
  double ndt_res_;
  int max_iter_;
  int omp_num_thread_;

  sensor_msgs::msg::Imu imu_;
  nav_msgs::msg::Odometry odom_;
};

#endif
