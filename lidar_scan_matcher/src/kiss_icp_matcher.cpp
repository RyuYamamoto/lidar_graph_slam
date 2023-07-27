#include "lidar_scan_matcher/kiss_icp_matcher.hpp"

KissIcpMatcher::KissIcpMatcher(const rclcpp::NodeOptions & node_options)
: Node("kiss_icp_matcher", node_options)
{
  base_frame_id_ = declare_parameter<std::string>("base_frame_id");
  displacement_ = declare_parameter<double>("displacement");

  config_.max_range = declare_parameter<double>("max_range");
  config_.min_range = declare_parameter<double>("min_range");
  config_.deskew = declare_parameter<bool>("deskew");
  config_.voxel_size = declare_parameter<double>("voxel_size");
  config_.max_points_per_voxel = declare_parameter<int>("max_points_per_voxel");
  config_.initial_threshold = declare_parameter<double>("initial_threshold");
  config_.min_motion_th = declare_parameter<double>("min_motion_th");
  if (config_.max_range < config_.min_range) {
    RCLCPP_WARN(
      get_logger(), "[WARNING] max_range is smaller than min_range, setting min_range to 0.0");
    config_.min_range = 0.0;
  }

  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  kiss_icp_ = kiss_icp::pipeline::KissICP(config_);

  sensor_points_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&KissIcpMatcher::callback_cloud, this, std::placeholders::_1));

  scan_matcher_path_publisher_ =
    this->create_publisher<nav_msgs::msg::Path>("scan_matcher_path", 5);
  scan_matcher_odom_publisher_ =
    this->create_publisher<nav_msgs::msg::Odometry>("scan_matcher_odom", 5);
  key_frame_publisher_ =
    this->create_publisher<lidar_graph_slam_msgs::msg::KeyFrame>("key_frame", 5);
}

void KissIcpMatcher::callback_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto base_to_sensor_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  tf_buffer_.transform(*msg, *base_to_sensor_cloud_msg, base_frame_id_);

  const auto sensor_points = point_cloud2_to_eigen(base_to_sensor_cloud_msg);

  const auto timestamps = [&]() -> std::vector<double> {
    if (!config_.deskew) return {};
    return get_timestamps(base_to_sensor_cloud_msg);
  }();

  const auto & [frame, key_points] = kiss_icp_.RegisterFrame(sensor_points, timestamps);

  const auto pose = kiss_icp_.poses().back();

  const Eigen::Vector3d translation = pose.translation();
  const Eigen::Quaterniond orientation = pose.unit_quaternion();

  if(!is_initialized_) {
    lidar_graph_slam_msgs::msg::KeyFrame key_frame;
    key_frame.header = base_to_sensor_cloud_msg->header;
    key_frame.pose.position = tf2::toMsg(translation);
    key_frame.pose.orientation = tf2::toMsg(orientation);
    key_frame.cloud = *msg;
    key_frame.id = key_frame_id_++;
    key_frame_publisher_->publish(key_frame);
    is_initialized_ = true;
  }

  const double delta = (translation - previous_translation_).norm();
  if(displacement_ < delta) {
    previous_translation_ = translation;
    accumulate_distance_ += delta;

    lidar_graph_slam_msgs::msg::KeyFrame key_frame;
    key_frame.header = base_to_sensor_cloud_msg->header;
    key_frame.pose.position = tf2::toMsg(translation);
    key_frame.pose.orientation = tf2::toMsg(orientation);
    key_frame.accum_distance = accumulate_distance_;
    key_frame.cloud = *msg;
    key_frame.id = key_frame_id_++;
    key_frame_publisher_->publish(key_frame);
  }

  nav_msgs::msg::Odometry odometry;
  odometry.header.frame_id = "odom";
  odometry.child_frame_id = base_frame_id_;
  odometry.header.stamp = msg->header.stamp;
  odometry.pose.pose.position = tf2::toMsg(translation);
  odometry.pose.pose.orientation = tf2::toMsg(orientation);
  scan_matcher_odom_publisher_->publish(odometry);

  geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance;
  pose_with_covariance.header.frame_id = "map";
  pose_with_covariance.header.stamp = msg->header.stamp;
  pose_with_covariance.pose.pose.position = tf2::toMsg(translation);
  pose_with_covariance.pose.pose.orientation = tf2::toMsg(orientation);

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "map";
  pose_stamped.header.stamp = msg->header.stamp;
  pose_stamped.pose = pose_with_covariance.pose.pose;
  odometry_path_.poses.emplace_back(pose_stamped);
  odometry_path_.header = pose_stamped.header;
  scan_matcher_path_publisher_->publish(odometry_path_);

  publish_tf(pose_with_covariance.pose.pose, msg->header.stamp, "map", base_frame_id_);
}

geometry_msgs::msg::TransformStamped KissIcpMatcher::get_transform(
  const std::string source_frame, const std::string target_frame)
{
  geometry_msgs::msg::TransformStamped frame_transform;
  try {
    frame_transform = tf_buffer_.lookupTransform(
      target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    frame_transform.header.stamp = now();
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
  return frame_transform;
}

pcl::PointCloud<PointType>::Ptr KissIcpMatcher::transform_point_cloud(
  const pcl::PointCloud<PointType>::Ptr input_cloud_ptr,
  const geometry_msgs::msg::TransformStamped transform)
{
  pcl::PointCloud<PointType>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointType>);
  const Eigen::Affine3d frame_affine = tf2::transformToEigen(transform);
  const Eigen::Matrix4f frame_matrix = frame_affine.matrix().cast<float>();
  pcl::transformPointCloud(*input_cloud_ptr, *transform_cloud_ptr, frame_matrix);

  return transform_cloud_ptr;
}

pcl::PointCloud<PointType>::Ptr KissIcpMatcher::transform_point_cloud(
  const pcl::PointCloud<PointType>::Ptr input_cloud_ptr, const Eigen::Matrix4f transform_matrix)
{
  pcl::PointCloud<PointType>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::transformPointCloud(*input_cloud_ptr, *transform_cloud_ptr, transform_matrix);

  return transform_cloud_ptr;
}

void KissIcpMatcher::publish_tf(
  const geometry_msgs::msg::Pose pose, const rclcpp::Time stamp, const std::string frame_id,
  const std::string child_frame_id)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.header.stamp = stamp;
  transform_stamped.transform.translation.x = pose.position.x;
  transform_stamped.transform.translation.y = pose.position.y;
  transform_stamped.transform.translation.z = pose.position.z;
  transform_stamped.transform.rotation = pose.orientation;

  broadcaster_->sendTransform(transform_stamped);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(KissIcpMatcher)
