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

#include "graph_based_slam/graph_based_slam.hpp"

using namespace lidar_graph_slam_utils;

namespace
{
// Heavy messages (full map, full key frame array) are published only every Nth key frame to avoid
// overflowing the reliable transport (which otherwise closes the connection and stalls all output).
constexpr int kHeavyPublishInterval = 5;
// Leaf size [m] for down-sampling the map before publishing it for visualization.
constexpr float kMapPublishLeafSize = 0.2f;
}  // namespace

GraphBasedSLAM::GraphBasedSLAM(const rclcpp::NodeOptions & node_options)
: Node("graph_based_slam", node_options)
{
  search_radius_ = this->declare_parameter<double>("search_radius");
  score_threshold_ = this->declare_parameter<double>("score_threshold");
  search_for_candidate_threshold_ =
    this->declare_parameter<double>("search_for_candidate_threshold");
  accumulate_distance_threshold_ = this->declare_parameter<double>("accumulate_distance_threshold");
  search_key_frame_num_ = this->declare_parameter<int>("search_key_frame_num");

  key_frame_subscriber_ = this->create_subscription<lidar_graph_slam_msgs::msg::KeyFrame>(
    "key_frame", 5, std::bind(&GraphBasedSLAM::key_frame_callback, this, std::placeholders::_1));

  modified_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("modified_path", 5);
  candidate_key_frame_publisher_ =
    this->create_publisher<nav_msgs::msg::Path>("candidate_key_frame", 5);
  accepted_loop_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "accepted_loop_edges", rclcpp::QoS{1}.transient_local());
  modified_key_frame_publisher_ =
    this->create_publisher<lidar_graph_slam_msgs::msg::KeyFrameArray>("modified_key_frame", 5);
  modified_map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "modified_map", rclcpp::QoS{1}.transient_local());

  save_map_service_ = this->create_service<lidar_graph_slam_msgs::srv::SaveMap>(
    "save_map",
    std::bind(
      &GraphBasedSLAM::save_map_service, this, std::placeholders::_1, std::placeholders::_2));

  kd_tree_.reset(new pcl::KdTreeFLANN<PointType>());
  key_frame_point_.reset(new pcl::PointCloud<PointType>);
  map_.reset(new pcl::PointCloud<PointType>);

  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.1;
  parameters.relinearizeSkip = 1;
  optimizer_ = std::make_shared<gtsam::ISAM2>(parameters);

  voxel_grid_.setLeafSize(0.5, 0.5, 0.5);

  const std::string registration_method =
    this->declare_parameter<std::string>("registration_method");
  registration_ = get_registration(registration_method);

  // NOTE: gtsam::Pose3 tangent ordering is [rx, ry, rz, x, y, z] (rotation first).
  // Prior: tightly anchor the first key frame to fix the gauge freedom.
  gtsam::Vector prior_variance(6);
  prior_variance << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8;
  prior_noise_ = gtsam::noiseModel::Diagonal::Variances(prior_variance);

  // Odometry: scan-matching relative motion is good but NOT rigid. Using a moderate
  // covariance (rot ~0.6deg, trans ~5cm) lets loop-closure constraints redistribute the
  // accumulated drift. (Previously this reused the ultra-tight prior noise, which made the
  // odometry chain effectively rigid and caused loop closures to be ignored.)
  gtsam::Vector odometry_variance(6);
  odometry_variance << 1e-4, 1e-4, 1e-4, 2.5e-3, 2.5e-3, 2.5e-3;
  odometry_noise_ = gtsam::noiseModel::Diagonal::Variances(odometry_variance);

  // cyan LINE_LIST marker accumulating every accepted loop-closure edge for RViz debugging.
  // (cyan keeps it distinct from the green modified_path trajectory.)
  accepted_loop_marker_.header.frame_id = "map";
  accepted_loop_marker_.ns = "accepted_loop_edges";
  accepted_loop_marker_.id = 0;
  accepted_loop_marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
  accepted_loop_marker_.action = visualization_msgs::msg::Marker::ADD;
  accepted_loop_marker_.scale.x = 0.3;
  accepted_loop_marker_.color.g = 1.0;
  accepted_loop_marker_.color.b = 1.0;
  accepted_loop_marker_.color.a = 1.0;
  accepted_loop_marker_.pose.orientation.w = 1.0;

  const double rate = declare_parameter<double>("rate");
  optimization_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(rate).period(),
    std::bind(&GraphBasedSLAM::optimization_callback, this), optimization_callback_group_);
}

pcl::Registration<PointType, PointType>::Ptr GraphBasedSLAM::get_registration(
  const std::string & registration_method)
{
  pcl::Registration<PointType, PointType>::Ptr registration;

  if (registration_method == "FAST_GICP") {
    RCLCPP_INFO_STREAM(get_logger(), "registration: " << registration_method.c_str());
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

    registration = fast_gicp;
  } else if (registration_method == "NDT_OMP") {
    RCLCPP_INFO_STREAM(get_logger(), "registration: " << registration_method.c_str());
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
    ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    if (0 < omp_num_thread) ndt_omp->setNumThreads(omp_num_thread);

    registration = ndt_omp;
  } else if (registration_method == "GICP") {
    RCLCPP_INFO_STREAM(get_logger(), "registration: " << registration_method.c_str());
    pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>::Ptr gicp(
      new pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>());

    const double correspondence_distance =
      this->declare_parameter<double>("correspondence_distance");
    const double max_iteration = this->declare_parameter<int>("max_iteration");
    const double transformation_epsilon = this->declare_parameter<double>("transformation_epsilon");
    const double euclidean_fitness_epsilon =
      this->declare_parameter<double>("euclidean_fitness_epsilon");
    const int ransac_iteration = this->declare_parameter<int>("ransac_iteration");
    const int max_optimizer_iteration = this->declare_parameter<int>("max_optimizer_iteration");

    gicp->setMaxCorrespondenceDistance(correspondence_distance);
    gicp->setMaximumIterations(max_iteration);
    gicp->setMaximumOptimizerIterations(max_optimizer_iteration);
    gicp->setTransformationEpsilon(transformation_epsilon);
    gicp->setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    gicp->setRANSACIterations(ransac_iteration);

    registration = gicp;
  } else if (registration_method == "ICP") {
    pcl::IterativeClosestPoint<PointType, PointType>::Ptr icp(
      new pcl::IterativeClosestPoint<PointType, PointType>());
    icp->setMaxCorrespondenceDistance(30);
    icp->setMaximumIterations(100);
    icp->setTransformationEpsilon(1e-8);
    icp->setEuclideanFitnessEpsilon(1e-6);
    icp->setRANSACIterations(0);

    registration = icp;
  }

  return registration;
}

bool GraphBasedSLAM::detect_loop_with_accum_dist(
  const lidar_graph_slam_msgs::msg::KeyFrame & latest_key_frame,
  const lidar_graph_slam_msgs::msg::KeyFrameArray & key_frame_array,
  std::vector<lidar_graph_slam_msgs::msg::KeyFrame> & candidate_key_frame)
{
  const rclcpp::Time latest_stamp = latest_key_frame.header.stamp;
  const Eigen::Vector3d latest_pose{
    latest_key_frame.pose.position.x, latest_key_frame.pose.position.y,
    latest_key_frame.pose.position.z};
  const double latest_accum_dist = latest_key_frame.accum_distance;

  for (const auto & key_frame : key_frame_array.keyframes) {
    if ((latest_accum_dist - key_frame.accum_distance) < accumulate_distance_threshold_) {
      continue;
    }

    const Eigen::Vector3d key_frame_pose{
      key_frame.pose.position.x, key_frame.pose.position.y, key_frame.pose.position.z};

    const double key_frame_dist = (latest_pose - key_frame_pose).norm();
    if (key_frame_dist < search_for_candidate_threshold_) {
      candidate_key_frame.emplace_back(key_frame);
    }
  }

  if (candidate_key_frame.empty()) return false;

  return true;
}

bool GraphBasedSLAM::detect_loop_with_kd_tree(
  const lidar_graph_slam_msgs::msg::KeyFrame & latest_key_frame,
  const pcl::PointCloud<PointType>::Ptr key_frame_cloud,
  pcl::PointCloud<PointType>::Ptr & nearest_key_frame_cloud, int & closest_key_frame_id)
{
  rclcpp::Time latest_stamp = latest_key_frame.header.stamp;
  geometry_msgs::msg::Pose latest_pose = latest_key_frame.pose;

  kd_tree_->setInputCloud(key_frame_cloud);

  std::vector<int> indices;
  std::vector<float> dists;
  PointType latest_key_point;
  latest_key_point.x = latest_pose.position.x;
  latest_key_point.y = latest_pose.position.y;
  latest_key_point.z = latest_pose.position.z;
  kd_tree_->radiusSearch(latest_key_point, search_radius_, indices, dists);

  closest_key_frame_id = -1;
  for (auto indice : indices) {
    if (30.0 < (latest_stamp - key_frame_array_.keyframes[indice].header.stamp).seconds()) {
      closest_key_frame_id = indice;
      break;
    }
  }

  if (closest_key_frame_id == -1) {
    return false;
  }

  const int key_frame_size = key_frame_array_.keyframes.size();
  for (int idx = -search_key_frame_num_; idx <= search_key_frame_num_; idx++) {
    int key_frame_cloud_idx = closest_key_frame_id + idx;
    if (key_frame_cloud_idx < 0 or key_frame_size <= key_frame_cloud_idx) continue;

    pcl::PointCloud<PointType>::Ptr tmp_cloud(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(key_frame_array_.keyframes[key_frame_cloud_idx].cloud, *tmp_cloud);

    pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>);
    const Eigen::Matrix4f matrix =
      geometry_pose_to_matrix(key_frame_array_.keyframes[key_frame_cloud_idx].pose);
    transformed_cloud = transform_point_cloud(tmp_cloud, matrix);
    *nearest_key_frame_cloud += *transformed_cloud;
  }

  return true;
}

void GraphBasedSLAM::optimization_callback()
{
  // ---- Phase 1: take a snapshot under the lock (cheap: candidate search + copy clouds) ----
  int key_frame_size = 0;
  int min_id = -1;
  lidar_graph_slam_msgs::msg::KeyFrame latest_key_frame;
  geometry_msgs::msg::Pose candidate_pose;
  std::vector<lidar_graph_slam_msgs::msg::KeyFrame> candidate_region;  // for the target sub-map
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);

    if (key_frame_array_.keyframes.empty()) return;

    // Skip new loop detection until the previous closure has been applied by adjust_pose (on the
    // next key frame). Otherwise the timer keeps re-detecting the same candidate on still-drifted
    // poses and adds duplicate loop factors, over-constraining that location.
    if (is_loop_closed_) return;

    key_frame_size = key_frame_array_.keyframes.size();
    latest_key_frame = key_frame_array_.keyframes.back();

    const Eigen::Vector3d latest_position{
      latest_key_frame.pose.position.x, latest_key_frame.pose.position.y,
      latest_key_frame.pose.position.z};
    const double latest_accum_dist = latest_key_frame.accum_distance;

    double min_dist = std::numeric_limits<double>::max();
    for (int id = 0; id < key_frame_size; id++) {
      const auto & key_frame = key_frame_array_.keyframes[id];
      if ((latest_accum_dist - key_frame.accum_distance) < accumulate_distance_threshold_) continue;
      const Eigen::Vector3d key_frame_position{
        key_frame.pose.position.x, key_frame.pose.position.y, key_frame.pose.position.z};
      const double key_frame_dist = (latest_position - key_frame_position).norm();
      if (key_frame_dist < search_for_candidate_threshold_ && key_frame_dist < min_dist) {
        min_dist = key_frame_dist;
        min_id = id;
      }
    }
    if (min_id == -1) return;

    candidate_pose = key_frame_array_.keyframes[min_id].pose;

    // Copy the candidate-region key frames so the heavy sub-map build + registration below can run
    // without holding the lock (key frames are only appended, never removed, while is_loop_closed_
    // is false, so these indices and poses stay valid).
    for (int idx = -search_key_frame_num_; idx <= search_key_frame_num_; idx++) {
      const int j = min_id + idx;
      if (j < 0 or key_frame_size <= j) continue;
      candidate_region.emplace_back(key_frame_array_.keyframes[j]);
    }

    candidate_line_.poses.clear();
    geometry_msgs::msg::PoseStamped pose_from;
    pose_from.pose = candidate_pose;
    pose_from.header = key_frame_array_.keyframes[min_id].header;
    geometry_msgs::msg::PoseStamped pose_to;
    pose_to.pose = latest_key_frame.pose;
    pose_to.header = latest_key_frame.header;
    candidate_line_.poses.emplace_back(pose_from);
    candidate_line_.poses.emplace_back(pose_to);
    candidate_line_.header.frame_id = "map";
    candidate_line_.header.stamp = latest_key_frame.header.stamp;
  }
  candidate_key_frame_publisher_->publish(candidate_line_);

  // ---- Phase 2: heavy sub-map build + registration WITHOUT the lock (does not block ingestion) --
  pcl::PointCloud<PointType>::Ptr latest_cloud(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(latest_key_frame.cloud, *latest_cloud);
  pcl::PointCloud<PointType>::Ptr source_cloud =
    transform_point_cloud(latest_cloud, geometry_pose_to_matrix(latest_key_frame.pose));

  pcl::PointCloud<PointType>::Ptr nearest_key_frame_cloud(new pcl::PointCloud<PointType>);
  for (const auto & key_frame : candidate_region) {
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(key_frame.cloud, *cloud);
    *nearest_key_frame_cloud += *transform_point_cloud(cloud, geometry_pose_to_matrix(key_frame.pose));
  }

  pcl::PointCloud<PointType>::Ptr filtered_target(new pcl::PointCloud<PointType>);
  voxel_grid_.setInputCloud(nearest_key_frame_cloud);
  voxel_grid_.filter(*filtered_target);

  registration_->setInputTarget(filtered_target);
  registration_->setInputSource(source_cloud);
  pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>);
  registration_->align(*output_cloud);

  const Eigen::Matrix4f transform = registration_->getFinalTransformation();
  const double fitness_score = registration_->getFitnessScore();
  const bool has_converged = registration_->hasConverged();

  if (!has_converged or score_threshold_ < fitness_score) {
    RCLCPP_INFO(
      get_logger(), "loop candidate REJECTED (latest=%d, min_id=%d): converged=%d, fitness=%.4f (> %.4f)",
      key_frame_size - 1, min_id, has_converged, fitness_score, score_threshold_);
    return;
  }
  RCLCPP_INFO(
    get_logger(), "loop candidate ACCEPTED (latest=%d, min_id=%d): fitness=%.4f",
    key_frame_size - 1, min_id, fitness_score);

  // correct position / candidate position (computed from the snapshot, so self-consistent)
  const auto pose_from = geometry_pose_to_gtsam_pose(
    convert_matrix_to_pose(transform * geometry_pose_to_matrix(latest_key_frame.pose)));
  const auto pose_to = geometry_pose_to_gtsam_pose(candidate_pose);
  // Loop-closure noise scaled by the registration fitness score, with rotation and
  // translation treated separately and clamped to sane floors. Order: [rx, ry, rz, x, y, z].
  const double loop_translation_variance = std::max(1e-2, fitness_score);
  const double loop_rotation_variance = std::max(1e-3, fitness_score * 0.1);
  gtsam::Vector loop_variance(6);
  loop_variance << loop_rotation_variance, loop_rotation_variance, loop_rotation_variance,
    loop_translation_variance, loop_translation_variance, loop_translation_variance;
  gtsam::noiseModel::Diagonal::shared_ptr optimize_noise =
    gtsam::noiseModel::Diagonal::Variances(loop_variance);

  // ---- Phase 3: add the loop factor and optimize under the lock (cheap) ----
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
      key_frame_size - 1, min_id, pose_from.between(pose_to), optimize_noise));

    // visualize the accepted loop edge (latest key frame <-> matched candidate) in cyan.
    accepted_loop_marker_.points.emplace_back(latest_key_frame.pose.position);
    accepted_loop_marker_.points.emplace_back(candidate_pose.position);
    accepted_loop_marker_.header.stamp = now();
    visualization_msgs::msg::MarkerArray accepted_loop_markers;
    accepted_loop_markers.markers.emplace_back(accepted_loop_marker_);
    accepted_loop_publisher_->publish(accepted_loop_markers);

    optimizer_->update(graph_);
    optimizer_->update();
    graph_.resize(0);

    is_loop_closed_ = true;
  }
}

void GraphBasedSLAM::key_frame_callback(const lidar_graph_slam_msgs::msg::KeyFrame::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  if (!is_initialized_key_frame_) is_initialized_key_frame_ = true;

  auto key_frame_size = key_frame_array_.keyframes.size();
  // Raw scan-matcher global pose of the current key frame.
  auto current_raw_pose = geometry_pose_to_gtsam_pose(msg->pose);
  if (key_frame_array_.keyframes.empty()) {
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, current_raw_pose, prior_noise_));
    initial_estimate_.insert(0, current_raw_pose);
  } else {
    // Odometry edge measurement must be the relative motion between consecutive RAW
    // scan-matcher poses. Using the optimized previous pose (key_frame_array_) would fold the
    // loop-closure correction back into the measurement and corrupt the graph after a closure.
    auto previous_raw_pose =
      geometry_pose_to_gtsam_pose(key_frame_raw_array_.keyframes.back().pose);
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
      key_frame_size - 1, key_frame_size, previous_raw_pose.between(current_raw_pose),
      odometry_noise_));
    initial_estimate_.insert(key_frame_size, current_raw_pose);
  }

  optimizer_->update(graph_, initial_estimate_);
  optimizer_->update();

  graph_.resize(0);
  initial_estimate_.clear();

  auto current_estimate = optimizer_->calculateEstimate();
  auto estimated_pose = current_estimate.at<gtsam::Pose3>(current_estimate.size() - 1);

  lidar_graph_slam_msgs::msg::KeyFrame key_frame;
  key_frame.header = msg->header;
  key_frame.cloud = msg->cloud;
  key_frame.pose = gtsam_pose_to_geometry_pose(estimated_pose);
  key_frame.accum_distance = msg->accum_distance;
  key_frame.id = msg->id;
  key_frame_array_.keyframes.emplace_back(key_frame);

  PointType key_frame_point;
  key_frame_point.x = key_frame.pose.position.x;
  key_frame_point.y = key_frame.pose.position.y;
  key_frame_point.z = key_frame.pose.position.z;
  key_frame_point_->points.emplace_back(key_frame_point);

  key_frame_raw_array_.keyframes.emplace_back(*msg);

  bool loop_applied = false;
  if (is_loop_closed_) {
    // loop closure shifts every key frame pose, so the cached map must be rebuilt from scratch.
    adjust_pose();
    rebuild_map();
    is_loop_closed_ = false;
    loop_applied = true;
  } else {
    // no loop closure: only the newest key frame is new, so append it incrementally (O(1)).
    append_key_frame_to_map(key_frame);
  }

  update_estimate_path();  // small message, publish every key frame

  // The full map and full key frame array are large and grow over time; publishing them every key
  // frame floods the reliable transport. Throttle them, but always publish right after a loop
  // closure so the corrected map/poses are shown promptly.
  const bool publish_heavy =
    loop_applied || (key_frame_array_.keyframes.size() % kHeavyPublishInterval == 0);
  if (publish_heavy) {
    publish_map();
    modified_key_frame_publisher_->publish(key_frame_array_);
  }
}

pcl::PointCloud<PointType>::Ptr GraphBasedSLAM::transform_point_cloud(
  const pcl::PointCloud<PointType>::Ptr input_cloud_ptr, const Eigen::Matrix4f & transform_matrix)
{
  pcl::PointCloud<PointType>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::transformPointCloud(*input_cloud_ptr, *transform_cloud_ptr, transform_matrix);

  return transform_cloud_ptr;
}

void GraphBasedSLAM::adjust_pose()
{
  auto current_estimate = optimizer_->calculateEstimate();

  double max_shift = 0.0;
  double sum_shift = 0.0;

  for (std::size_t idx = 0; idx < current_estimate.size(); idx++) {
    const auto & previous_position = key_frame_array_.keyframes[idx].pose.position;
    const gtsam::Pose3 optimized = current_estimate.at<gtsam::Pose3>(idx);

    const double shift = std::hypot(
      optimized.x() - previous_position.x, optimized.y() - previous_position.y,
      optimized.z() - previous_position.z);
    max_shift = std::max(max_shift, shift);
    sum_shift += shift;

    key_frame_array_.keyframes[idx].pose = gtsam_pose_to_geometry_pose(optimized);

    PointType key_frame_point;
    key_frame_point.x = key_frame_array_.keyframes[idx].pose.position.x;
    key_frame_point.y = key_frame_array_.keyframes[idx].pose.position.y;
    key_frame_point.z = key_frame_array_.keyframes[idx].pose.position.z;
    key_frame_point_->points[idx] = key_frame_point;
  }

  const double mean_shift = current_estimate.empty()
                              ? 0.0
                              : sum_shift / static_cast<double>(current_estimate.size());
  RCLCPP_INFO(
    get_logger(), "loop closure correction: max=%.3f m, mean=%.3f m over %zu key frames", max_shift,
    mean_shift, current_estimate.size());
}

void GraphBasedSLAM::update_estimate_path()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = now();
  for (auto & key_frame : key_frame_array_.keyframes) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = key_frame.header;
    pose_stamped.pose = key_frame.pose;
    path.poses.emplace_back(pose_stamped);
  }
  modified_path_publisher_->publish(path);
}

void GraphBasedSLAM::append_key_frame_to_map(
  const lidar_graph_slam_msgs::msg::KeyFrame & key_frame)
{
  pcl::PointCloud<PointType>::Ptr key_frame_cloud(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(key_frame.cloud, *key_frame_cloud);

  const Eigen::Matrix4f matrix = geometry_pose_to_matrix(key_frame.pose);
  *map_ += *transform_point_cloud(key_frame_cloud, matrix);
}

void GraphBasedSLAM::rebuild_map()
{
  map_->clear();
  for (const auto & key_frame : key_frame_array_.keyframes) {
    append_key_frame_to_map(key_frame);
  }
}

void GraphBasedSLAM::publish_map()
{
  // Down-sample before publishing so the visualization message size stays bounded as the map grows
  // (the full-resolution map is kept in map_ and used by the save_map service).
  pcl::PointCloud<PointType>::Ptr downsampled_map(new pcl::PointCloud<PointType>);
  pcl::VoxelGrid<PointType> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(kMapPublishLeafSize, kMapPublishLeafSize, kMapPublishLeafSize);
  voxel_grid_filter.setInputCloud(map_);
  voxel_grid_filter.filter(*downsampled_map);

  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*downsampled_map, map_msg);
  map_msg.header.frame_id = "map";
  map_msg.header.stamp = now();
  modified_map_publisher_->publish(map_msg);
}

bool GraphBasedSLAM::save_map_service(
  const lidar_graph_slam_msgs::srv::SaveMap::Request::SharedPtr req,
  lidar_graph_slam_msgs::srv::SaveMap::Response::SharedPtr res)
{
  // Snapshot the key frames under the lock, then build the map from the copy without holding it
  // (the service runs in a different callback group than the SLAM callbacks under a MT executor).
  lidar_graph_slam_msgs::msg::KeyFrameArray key_frames;
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    key_frames = key_frame_array_;
  }

  pcl::PointCloud<PointType>::Ptr map(new pcl::PointCloud<PointType>);
  for (std::size_t idx = 0; idx < key_frames.keyframes.size(); idx++) {
    pcl::PointCloud<PointType>::Ptr key_frame_cloud(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(key_frames.keyframes[idx].cloud, *key_frame_cloud);

    pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>);
    const Eigen::Matrix4f matrix = geometry_pose_to_matrix(key_frames.keyframes[idx].pose);
    transformed_cloud = transform_point_cloud(key_frame_cloud, matrix);

    *map += *transformed_cloud;
  }

  pcl::PointCloud<PointType>::Ptr map_cloud(new pcl::PointCloud<PointType>);

  if (req->resolution <= 0.0) {
    map_cloud = map;
  } else {
    pcl::VoxelGrid<PointType> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(req->resolution, req->resolution, req->resolution);
    voxel_grid_filter.setInputCloud(map);
    voxel_grid_filter.filter(*map_cloud);
  }

  map_cloud->header.frame_id = "map";
  int ret = pcl::io::savePCDFile(req->path, *map_cloud);
  res->ret = (ret == 0);

  return true;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(GraphBasedSLAM)
