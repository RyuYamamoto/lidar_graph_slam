#include "lidar_scan_matcher/lidar_scan_matcher.hpp"

LidarScanMatcher::LidarScanMatcher() : Node("lidar_scan_matcher_node")
{
  sensor_points_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&LidarScanMatcher::callback_cloud, this, std::placeholders::_1));
}

void LidarScanMatcher::callback_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<PointType>::Ptr input_cloud_ptr(new pcl::PointCloud<PointType>);
}

pcl::PointCloud<PointType>::Ptr LidarScanMatcher::transform_point_cloud(
  const std::string source_frame, const std::string target_frame, const rclcpp::Time stamp,
  pcl::PointCloud<PointType>::Ptr input_cloud_ptr)
{
  pcl::PointCloud<PointType>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointType>);
  geometry_msgs::msg::TransformStamped frame_transform;
  try {
    frame_transform =
      tf_buffer_.lookupTransform(source_frame, target_frame, stamp, tf2::durationFromSec(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    frame_transform.header.stamp = stamp;
    frame_transform.header.frame_id = source_frame;
    frame_transform.child_frame_id = target_frame;
    frame_transform.transform.translation.x = 0.0;
    frame_transform.transform.translation.y = 0.0;
    frame_transform.transform.translation.z = 0.0;
    frame_transform.transform.rotation.w = 1.0;
    frame_transform.transform.rotation.x = 0.0;
    frame_transform.transform.rotation.y = 0.0;
    frame_transform.transform.rotation.z = 0.0;
  }
  const Eigen::Affine3d frame_affine = tf2::transformToEigen(frame_transform);
  const Eigen::Matrix4f frame_matrix = frame_affine.matrix().cast<float>();
  pcl::transformPointCloud(*input_cloud_ptr, *transform_cloud_ptr, frame_matrix);

  return transform_cloud_ptr;
}