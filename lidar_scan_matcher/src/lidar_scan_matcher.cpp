#include "lidar_scan_matcher/lidar_scan_matcher.hpp"

LidarScanMatcher::LidarScanMatcher(const rclcpp::NodeOptions & node_options)
: Node("lidar_scan_matcher_node", node_options)
{
  transformation_epsilon_ = this->declare_parameter<double>("transformation_epsilon");
  step_size_ = this->declare_parameter<double>("step_size");
  ndt_resolution_ = this->declare_parameter<double>("ndt_resolution");
  max_iteration_ = this->declare_parameter<int>("max_iteration");
  omp_num_thread_ = this->declare_parameter<int>("omp_num_thread");
  base_frame_id_ = this->declare_parameter<std::string>("base_frame_id");

  registration_ = std::make_shared<pclomp::NormalDistributionsTransform<PointType, PointType>>();

  registration_->setTransformationEpsilon(transformation_epsilon_);
  registration_->setStepSize(step_size_);
  registration_->setResolution(ndt_resolution_);
  registration_->setMaximumIterations(max_iteration_);
  registration_->setNeighborhoodSearchMethod(pclomp::KDTREE);
  if (0 < omp_num_thread_) registration_->setNumThreads(omp_num_thread_);

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

  pcl::fromROSMsg(*msg, *input_cloud_ptr);

  const geometry_msgs::msg::TransformStamped base_to_sensor_transform =
    get_transform(base_frame_id_, msg->header.frame_id);
  transform_cloud_ptr = transform_point_cloud(input_cloud_ptr, base_to_sensor_transform);

  if (!target_cloud_) {
    prev_translation_.setIdentity();
    target_cloud_.reset(new pcl::PointCloud<PointType>);
    target_cloud_ = transform_cloud_ptr;
    registration_->setInputTarget(target_cloud_);
  }

  registration_->setInputSource(transform_cloud_ptr);

  pcl::PointCloud<PointType>::Ptr aligned_cloud_ptr(new pcl::PointCloud<PointType>);
  registration_->align(*aligned_cloud_ptr, prev_translation_);

  if (!registration_->hasConverged()) {
    RCLCPP_ERROR(get_logger(), "LiDAR Scan Matching has not Converged.");
    return;
  }

  translation_ = registration_->getFinalTransformation();
  transform_cloud_ptr = transform_point_cloud(
    input_cloud_ptr, translation_ * convert_transform_to_matrix(base_to_sensor_transform));

  const Eigen::Vector3d current_position = translation_.block<3, 1>(0, 3).cast<double>();
  const Eigen::Vector3d previous_position = key_frame_.block<3, 1>(0, 3).cast<double>();
  const double delta = (current_position - previous_position).norm();
  if (0.2 <= delta) {
    key_frame_ = translation_;
    target_cloud_ = transform_cloud_ptr;
    registration_->setInputTarget(target_cloud_);
  }

  nav_msgs::msg::Odometry odometry;

  Eigen::Vector3d translation = translation_.block<3, 1>(0, 3).cast<double>();
  Eigen::Matrix3d rotation = translation_.block<3, 3>(0, 0).cast<double>();
  Eigen::Quaterniond quat(rotation);

  odometry.header.frame_id = "odom";
  odometry.child_frame_id = base_frame_id_;
  odometry.header.stamp = msg->header.stamp;
  odometry.pose.pose.position.x = translation.x();
  odometry.pose.pose.position.y = translation.y();
  odometry.pose.pose.position.z = translation.z();
  odometry.pose.pose.orientation = tf2::toMsg(quat);

  scan_matcher_odom_publisher_->publish(odometry);

  prev_translation_ = translation_;
}

geometry_msgs::msg::TransformStamped LidarScanMatcher::get_transform(
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

pcl::PointCloud<PointType>::Ptr LidarScanMatcher::transform_point_cloud(
  const pcl::PointCloud<PointType>::Ptr input_cloud_ptr,
  const geometry_msgs::msg::TransformStamped transform)
{
  pcl::PointCloud<PointType>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointType>);
  const Eigen::Affine3d frame_affine = tf2::transformToEigen(transform);
  const Eigen::Matrix4f frame_matrix = frame_affine.matrix().cast<float>();
  pcl::transformPointCloud(*input_cloud_ptr, *transform_cloud_ptr, frame_matrix);

  return transform_cloud_ptr;
}

pcl::PointCloud<PointType>::Ptr LidarScanMatcher::transform_point_cloud(
  const pcl::PointCloud<PointType>::Ptr input_cloud_ptr, const Eigen::Matrix4f transform_matrix)
{
  pcl::PointCloud<PointType>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::transformPointCloud(*input_cloud_ptr, *transform_cloud_ptr, transform_matrix);

  return transform_cloud_ptr;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(LidarScanMatcher)