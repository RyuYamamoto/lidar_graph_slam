#include "lidar_scan_matcher/lidar_scan_matcher.hpp"

LidarScanMatcher::LidarScanMatcher(const rclcpp::NodeOptions & node_options)
: Node("lidar_scan_matcher_node", node_options)
{
  sensor_points_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&LidarScanMatcher::callback_cloud, this, std::placeholders::_1));

  scan_matcher_odom_publisher_ =
    this->create_publisher<nav_msgs::msg::Odometry>("scan_matcher_odom", 5);
}

void LidarScanMatcher::callback_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<PointType>::Ptr input_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointType>);

  transform_cloud_ptr =
    transform_point_cloud(base_frame_id_, msg->header.frame_id, msg->header.stamp, input_cloud_ptr);

  if (!target_cloud_) {
    key_frame_.setIdentity();
    target_cloud_.reset(new pcl::PointCloud<PointType>);
    target_cloud_ = transform_cloud_ptr;
    registration_->setInputTarget(target_cloud_);
  }

  registration_->setInputSource(transform_cloud_ptr);

  pcl::PointCloud<PointType>::Ptr aligned_cloud_ptr(new pcl::PointCloud<PointType>);
  registration_->align(*aligned_cloud_ptr, key_frame_);

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

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(LidarScanMatcher)