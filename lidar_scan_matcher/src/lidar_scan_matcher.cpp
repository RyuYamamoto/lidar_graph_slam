#include "lidar_scan_matcher/lidar_scan_matcher.hpp"

using namespace lidar_graph_slam_utils;

LidarScanMatcher::LidarScanMatcher(const rclcpp::NodeOptions & node_options)
: Node("lidar_scan_matcher_node", node_options)
{
  base_frame_id_ = this->declare_parameter<std::string>("base_frame_id");
  displacement_ = this->declare_parameter<double>("displacement");
  max_scan_accumulate_num_ = this->declare_parameter<int>("max_scan_accumulate_num");

  const std::string registration_method =
    this->declare_parameter<std::string>("registration_method");

  if (registration_method == "FAST_GICP") {
    fast_gicp::FastGICP<PointType, PointType>::Ptr fast_gicp(
      new fast_gicp::FastGICP<PointType, PointType>);

    const int max_iteration = this->declare_parameter<int>("max_iteration");
    const int omp_num_thread = this->declare_parameter<int>("omp_num_thread");
    const int correspondence_randomness = this->declare_parameter<int>("correspondence_randomness");
    const double transformation_epsilon = this->declare_parameter<double>("transformation_epsilon");
    const double max_correspondence_distance =
      this->declare_parameter<double>("max_correspondence_distance");

    fast_gicp->setCorrespondenceRandomness(correspondence_randomness);
    fast_gicp->setMaximumIterations(max_iteration);
    fast_gicp->setTransformationEpsilon(transformation_epsilon);
    fast_gicp->setMaxCorrespondenceDistance(max_correspondence_distance);
    if (0 < omp_num_thread) fast_gicp->setNumThreads(omp_num_thread);

    registration_ = fast_gicp;
  } else if (registration_method == "NDT_OMP") {
    pclomp::NormalDistributionsTransform<PointType, PointType>::Ptr ndt_omp(
      new pclomp::NormalDistributionsTransform<PointType, PointType>);

    const double transformation_epsilon = this->declare_parameter<double>("transformation_epsilon");
    const double step_size = this->declare_parameter<double>("step_size");
    const double ndt_resolution = this->declare_parameter<double>("ndt_resolution");
    const int max_iteration = this->declare_parameter<int>("max_iteration");
    const int omp_num_thread = this->declare_parameter<int>("omp_num_thread");

    ndt_omp->setTransformationEpsilon(transformation_epsilon);
    ndt_omp->setStepSize(step_size);
    ndt_omp->setResolution(ndt_resolution);
    ndt_omp->setMaximumIterations(max_iteration);
    ndt_omp->setNeighborhoodSearchMethod(pclomp::KDTREE);
    if (0 < omp_num_thread) ndt_omp->setNumThreads(omp_num_thread);

    registration_ = ndt_omp;
  }

  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  sensor_points_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&LidarScanMatcher::callback_cloud, this, std::placeholders::_1));

  front_end_map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "local_map", rclcpp::QoS{1}.transient_local());
  scan_matcher_path_publisher_ =
    this->create_publisher<nav_msgs::msg::Path>("scan_matcher_path", 5);
  scan_matcher_pose_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("scan_matcher_pose", 5);
  scan_matcher_odom_publisher_ =
    this->create_publisher<nav_msgs::msg::Odometry>("scan_matcher_odom", 5);
  key_frame_publisher_ =
    this->create_publisher<lidar_graph_slam_msgs::msg::KeyFrame>("key_frame", 5);
}

void LidarScanMatcher::callback_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<PointType>::Ptr input_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointType>);

  pcl::fromROSMsg(*msg, *input_cloud_ptr);

  const geometry_msgs::msg::TransformStamped base_to_sensor_transform =
    get_transform(msg->header.frame_id, base_frame_id_);
  transform_cloud_ptr = transform_point_cloud(input_cloud_ptr, base_to_sensor_transform);

  if (!target_cloud_) {
    prev_translation_.setIdentity();
    key_frame_.setIdentity();
    target_cloud_.reset(new pcl::PointCloud<PointType>);
    target_cloud_->header.frame_id = "map";

    lidar_graph_slam_msgs::msg::KeyFrame key_frame;
    key_frame.pose = lidar_graph_slam_utils::convert_matrix_to_pose(prev_translation_);
    pcl::toROSMsg(*input_cloud_ptr, key_frame.cloud);
    key_frame_array_.keyframes.emplace_back(key_frame);

    *target_cloud_ += *transform_cloud_ptr;
    registration_->setInputTarget(target_cloud_);

    sensor_msgs::msg::PointCloud2 local_map;
    local_map.header.frame_id = "map";
    local_map.header.stamp = msg->header.stamp;
    pcl::toROSMsg(*target_cloud_, local_map);
    front_end_map_publisher_->publish(local_map);
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
    input_cloud_ptr,
    translation_ * lidar_graph_slam_utils::convert_transform_to_matrix(base_to_sensor_transform));

  prev_translation_ = translation_;

  const Eigen::Vector3d current_position = translation_.block<3, 1>(0, 3).cast<double>();
  const Eigen::Vector3d previous_position = key_frame_.block<3, 1>(0, 3).cast<double>();
  const double delta = (current_position - previous_position).norm();
  if (displacement_ <= delta) {
    key_frame_ = translation_;

    target_cloud_->points.clear();

    lidar_graph_slam_msgs::msg::KeyFrame key_frame;
    key_frame.header.frame_id = "map";
    key_frame.header.stamp = msg->header.stamp;
    key_frame.pose = lidar_graph_slam_utils::convert_matrix_to_pose(key_frame_);
    pcl::toROSMsg(*input_cloud_ptr, key_frame.cloud);
    key_frame_array_.keyframes.emplace_back(key_frame);

    const int sub_map_size = key_frame_array_.keyframes.size();
    for (int idx = 0; idx < max_scan_accumulate_num_; idx++) {
      if ((sub_map_size - 1 - idx) < 0) continue;
      pcl::PointCloud<PointType>::Ptr key_frame_cloud(new pcl::PointCloud<PointType>);
      pcl::fromROSMsg(key_frame_array_.keyframes[sub_map_size - 1 - idx].cloud, *key_frame_cloud);

      pcl::PointCloud<PointType>::Ptr transformed_key_cloud(new pcl::PointCloud<PointType>);
      const Eigen::Matrix4f matrix =
        geometry_pose_to_matrix(key_frame_array_.keyframes[sub_map_size - 1 - idx].pose);
      transformed_key_cloud = transform_point_cloud(
        key_frame_cloud,
        matrix * lidar_graph_slam_utils::convert_transform_to_matrix(base_to_sensor_transform));
      *target_cloud_ += *transformed_key_cloud;
    }

    registration_->setInputTarget(target_cloud_);

    sensor_msgs::msg::PointCloud2 local_map;
    local_map.header.frame_id = "map";
    local_map.header.stamp = msg->header.stamp;
    pcl::toROSMsg(*target_cloud_, local_map);
    front_end_map_publisher_->publish(local_map);

    publish_key_frame(key_frame);
  }

  Eigen::Vector3d translation = translation_.block<3, 1>(0, 3).cast<double>();
  Eigen::Quaterniond quaternion(translation_.block<3, 3>(0, 0).cast<double>());

  nav_msgs::msg::Odometry odometry;
  odometry.header.frame_id = "odom";
  odometry.child_frame_id = base_frame_id_;
  odometry.header.stamp = msg->header.stamp;
  odometry.pose.pose.position = tf2::toMsg(translation);
  odometry.pose.pose.orientation = tf2::toMsg(quaternion);
  scan_matcher_odom_publisher_->publish(odometry);

  geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance;
  pose_with_covariance.header.frame_id = "map";
  pose_with_covariance.header.stamp = msg->header.stamp;
  pose_with_covariance.pose.pose.position = tf2::toMsg(translation);
  pose_with_covariance.pose.pose.orientation = tf2::toMsg(quaternion);
  scan_matcher_pose_publisher_->publish(pose_with_covariance);

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "map";
  pose_stamped.header.stamp = msg->header.stamp;
  pose_stamped.pose = pose_with_covariance.pose.pose;
  estimated_path_.poses.emplace_back(pose_stamped);
  estimated_path_.header = pose_stamped.header;
  scan_matcher_path_publisher_->publish(estimated_path_);

  publish_tf(pose_with_covariance.pose.pose, msg->header.stamp, "map", base_frame_id_);
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

void LidarScanMatcher::publish_key_frame(const lidar_graph_slam_msgs::msg::KeyFrame key_frame)
{
  key_frame_publisher_->publish(key_frame);
}

void LidarScanMatcher::publish_tf(
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
RCLCPP_COMPONENTS_REGISTER_NODE(LidarScanMatcher)
