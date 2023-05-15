#include "graph_based_slam/graph_based_slam.hpp"

using namespace lidar_graph_slam_utils;

GraphBasedSLAM::GraphBasedSLAM(const rclcpp::NodeOptions & node_options)
: Node("graph_based_slam", node_options)
{
  key_frame_subscriber_ = this->create_subscription<lidar_graph_slam_msgs::msg::KeyFrame>(
    "key_frame", 5, std::bind(&GraphBasedSLAM::key_frame_callback, this, std::placeholders::_1));

  odometry_key_frame_marker_publisher_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("odometry_key_frame_marker", 5);
  modified_key_frame_marker_publisher_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("modified_key_frame_marker", 5);
  modified_map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "modified_map", rclcpp::QoS{1}.transient_local());

  prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());

  const double rate = declare_parameter<double>("rate");
  timer_ = create_timer(
    this, get_clock(), rclcpp::Rate(rate).period(),
    std::bind(&GraphBasedSLAM::optimization_callback, this));
}

bool GraphBasedSLAM::detect_loop() { return true; }

void GraphBasedSLAM::optimization_callback()
{
  if (key_frame_array_.keyframes.empty()) return;

  if (detect_loop()) {
  }
}

void GraphBasedSLAM::key_frame_callback(const lidar_graph_slam_msgs::msg::KeyFrame::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!is_initialized_key_frame_) is_initialized_key_frame_ = true;

  auto key_frame_size = key_frame_array_.keyframes.size();
  auto latest_key_frame = geometry_pose_to_gtsam_pose(msg->pose);
  if (key_frame_array_.keyframes.empty()) {
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, latest_key_frame, prior_noise_));
    initial_estimate_.insert(0, latest_key_frame);
  } else {
    auto previous_key_frame = geometry_pose_to_gtsam_pose(key_frame_array_.keyframes.back().pose);
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
      key_frame_size - 1, key_frame_size, previous_key_frame.between(latest_key_frame),
      prior_noise_));
    initial_estimate_.insert(key_frame_size, latest_key_frame);
  }

  optimizer_.update(graph_, initial_estimate_);
  optimizer_.update();

  graph_.resize(0);
  initial_estimate_.clear();

  auto current_estimate = optimizer_.calculateEstimate();
  auto estimated_pose = current_estimate.at<gtsam::Pose3>(current_estimate.size() - 1);

  lidar_graph_slam_msgs::msg::KeyFrame key_frame;
  key_frame.cloud = msg->cloud;
  key_frame.pose = gtsam_pose_to_geometry_pose(estimated_pose);
  key_frame_array_.keyframes.emplace_back(key_frame);

  for (std::size_t idx = 0; idx < current_estimate.size(); idx++) {
    key_frame_array_.keyframes[idx].pose =
      gtsam_pose_to_geometry_pose(current_estimate.at<gtsam::Pose3>(idx));
  }

  key_frame_raw_array_.keyframes.emplace_back(*msg);

  publish_key_frame_marker();

  pcl::PointCloud<PointType>::Ptr map(new pcl::PointCloud<PointType>);
  for (std::size_t idx = 0; idx < key_frame_array_.keyframes.size(); idx++) {
    pcl::PointCloud<PointType>::Ptr key_frame_cloud(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(key_frame_array_.keyframes[idx].cloud, *key_frame_cloud);

    pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>);
    const Eigen::Matrix4f matrix = geometry_pose_to_matrix(key_frame_array_.keyframes[idx].pose);
    transformed_cloud = transform_point_cloud(key_frame_cloud, matrix);

    *map += *transformed_cloud;
  }

  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*map, map_msg);
  map_msg.header.frame_id = "map";
  map_msg.header.stamp = now();
  modified_map_publisher_->publish(map_msg);
}

pcl::PointCloud<PointType>::Ptr GraphBasedSLAM::transform_point_cloud(
  const pcl::PointCloud<PointType>::Ptr input_cloud_ptr, const Eigen::Matrix4f transform_matrix)
{
  pcl::PointCloud<PointType>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::transformPointCloud(*input_cloud_ptr, *transform_cloud_ptr, transform_matrix);

  return transform_cloud_ptr;
}

void GraphBasedSLAM::publish_key_frame_marker()
{
  const rclcpp::Time current_stamp = now();

  if (!key_frame_array_.keyframes.empty()) {
    visualization_msgs::msg::MarkerArray marker_array;
    for (std::size_t idx = 0; idx < key_frame_array_.keyframes.size(); idx++) {
      const lidar_graph_slam_msgs::msg::KeyFrame key_frame = key_frame_array_.keyframes[idx];
      visualization_msgs::msg::Marker marker = create_marker(
        key_frame.pose, current_stamp, visualization_msgs::msg::Marker::SPHERE, idx, "",
        create_scale(1.0, 1.0, 1.0), create_color(0.7, 0.0, 0.0, 1.0));
      marker_array.markers.emplace_back(marker);
    }
    modified_key_frame_marker_publisher_->publish(marker_array);
  }

  if (!key_frame_raw_array_.keyframes.empty()) {
    visualization_msgs::msg::MarkerArray marker_array;
    for (std::size_t idx = 0; idx < key_frame_raw_array_.keyframes.size(); idx++) {
      const lidar_graph_slam_msgs::msg::KeyFrame key_frame = key_frame_array_.keyframes[idx];
      visualization_msgs::msg::Marker marker = create_marker(
        key_frame.pose, current_stamp, visualization_msgs::msg::Marker::SPHERE, idx, "",
        create_scale(1.0, 1.0, 1.0), create_color(0.7, 0.0, 1.0, 0.0));
      marker_array.markers.emplace_back(marker);
    }
    odometry_key_frame_marker_publisher_->publish(marker_array);
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(GraphBasedSLAM)
