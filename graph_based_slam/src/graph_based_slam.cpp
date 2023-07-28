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
  modified_key_frame_publisher_ =
    this->create_publisher<lidar_graph_slam_msgs::msg::KeyFrameArray>("modified_key_frame", 5);
  modified_map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "modified_map", rclcpp::QoS{1}.transient_local());

  kd_tree_.reset(new pcl::KdTreeFLANN<PointType>());
  key_frame_point_.reset(new pcl::PointCloud<PointType>);

  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.1;
  parameters.relinearizeSkip = 1;
  optimizer_ = std::make_shared<gtsam::ISAM2>(parameters);

  voxel_grid_.setLeafSize(0.5, 0.5, 0.5);

  const std::string registration_method =
    this->declare_parameter<std::string>("registration_method");
  registration_ = get_registration(registration_method);

  gtsam::Vector Vector6(6);
  Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
  prior_noise_ = gtsam::noiseModel::Diagonal::Variances(Vector6);

  const double rate = declare_parameter<double>("rate");
  timer_ = create_timer(
    this, get_clock(), rclcpp::Rate(rate).period(),
    std::bind(&GraphBasedSLAM::optimization_callback, this));
}

pcl::Registration<PointType, PointType>::Ptr GraphBasedSLAM::get_registration(
  const std::string registration_method)
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
  const lidar_graph_slam_msgs::msg::KeyFrame latest_key_frame,
  const lidar_graph_slam_msgs::msg::KeyFrameArray key_frame_array,
  std::vector<lidar_graph_slam_msgs::msg::KeyFrame> & candidate_key_frame)
{
  const rclcpp::Time latest_stamp = latest_key_frame.header.stamp;
  const Eigen::Vector3d latest_pose{
    latest_key_frame.pose.position.x, latest_key_frame.pose.position.y,
    latest_key_frame.pose.position.z};
  const double latest_accum_dist = latest_key_frame.accum_distance;

  const int key_frame_size = key_frame_array.keyframes.size();

  for (auto key_frame : key_frame_array.keyframes) {
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
  const lidar_graph_slam_msgs::msg::KeyFrame latest_key_frame,
  // const lidar_graph_slam_msgs::msg::KeyFrameArray key_frame_array,
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
  if (key_frame_array_.keyframes.empty()) return;

  std::lock_guard<std::mutex> lock(optimize_thread_mutex_);

  const int key_frame_size = key_frame_array_.keyframes.size();
  const auto latest_key_frame = key_frame_array_.keyframes.back();
  pcl::PointCloud<PointType>::Ptr nearest_key_frame_cloud(new pcl::PointCloud<PointType>);

  pcl::PointCloud<PointType>::Ptr latest_key_frame_cloud(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(latest_key_frame.cloud, *latest_key_frame_cloud);
  pcl::PointCloud<PointType>::Ptr transformed_key_frame_cloud(new pcl::PointCloud<PointType>);
  const Eigen::Matrix4f matrix = geometry_pose_to_matrix(latest_key_frame.pose);
  transformed_key_frame_cloud = transform_point_cloud(latest_key_frame_cloud, matrix);

  Eigen::Matrix4f correct_frame;

  double min_dist = std::numeric_limits<double>::max();
  int min_id = -1;

  const Eigen::Vector3d latest_pose{
    latest_key_frame.pose.position.x, latest_key_frame.pose.position.y,
    latest_key_frame.pose.position.z};
  const double latest_accum_dist = latest_key_frame.accum_distance;

  for (int id = 0; id < key_frame_size; id++) {
    auto key_frame = key_frame_array_.keyframes[id];
    if ((latest_accum_dist - key_frame.accum_distance) < accumulate_distance_threshold_) {
      continue;
    }

    const Eigen::Vector3d key_frame_pose{
      key_frame.pose.position.x, key_frame.pose.position.y, key_frame.pose.position.z};

    const double key_frame_dist = (latest_pose - key_frame_pose).norm();
    if (key_frame_dist < search_for_candidate_threshold_) {
      if (key_frame_dist < min_dist) {
        min_dist = key_frame_dist;
        min_id = id;
      }
    }
  }

  if (min_id == -1) return;

  {
    geometry_msgs::msg::PoseStamped pose_to;
    pose_to.pose = latest_key_frame.pose;
    pose_to.header = latest_key_frame.header;
    geometry_msgs::msg::PoseStamped pose_from;
    pose_from.pose = key_frame_array_.keyframes[min_id].pose;
    pose_from.header = key_frame_array_.keyframes[min_id].header;
    candidate_line_.poses.emplace_back(pose_from);
    candidate_line_.poses.emplace_back(pose_to);
    candidate_line_.header.frame_id = "map";
    candidate_line_.header.stamp = latest_key_frame.header.stamp;
  }

  for (int idx = -search_key_frame_num_; idx <= search_key_frame_num_; idx++) {
    int key_frame_cloud_idx = min_id + idx;
    if (key_frame_cloud_idx < 0 or key_frame_size <= key_frame_cloud_idx) continue;

    pcl::PointCloud<PointType>::Ptr tmp_cloud(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(key_frame_array_.keyframes[key_frame_cloud_idx].cloud, *tmp_cloud);

    pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>);
    const Eigen::Matrix4f matrix =
      geometry_pose_to_matrix(key_frame_array_.keyframes[key_frame_cloud_idx].pose);
    transformed_cloud = transform_point_cloud(tmp_cloud, matrix);
    *nearest_key_frame_cloud += *transformed_cloud;
  }

  pcl::PointCloud<PointType>::Ptr tmp_cloud(new pcl::PointCloud<PointType>);
  voxel_grid_.setInputCloud(nearest_key_frame_cloud);
  voxel_grid_.filter(*tmp_cloud);

  registration_->setInputTarget(tmp_cloud);
  registration_->setInputSource(transformed_key_frame_cloud);
  pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>);
  registration_->align(*output_cloud);

  const Eigen::Matrix4f transform = registration_->getFinalTransformation();
  const double fitness_score = registration_->getFitnessScore();
  const bool has_converged = registration_->hasConverged();

  RCLCPP_INFO_STREAM(get_logger(), "fitness score: " << fitness_score);
  RCLCPP_INFO_STREAM(get_logger(), "min_id: " << min_id);
  candidate_key_frame_publisher_->publish(candidate_line_);

  if (!has_converged or score_threshold_ < fitness_score) return;

  // correct position
  auto pose_from = geometry_pose_to_gtsam_pose(
    convert_matrix_to_pose(transform * geometry_pose_to_matrix(latest_key_frame.pose)));
  // candidate position
  auto pose_to = geometry_pose_to_gtsam_pose(key_frame_array_.keyframes[min_id].pose);
  gtsam::Vector Vector6(6);
  Vector6 << fitness_score, fitness_score, fitness_score, fitness_score, fitness_score,
    fitness_score;
  gtsam::noiseModel::Diagonal::shared_ptr optimize_noise =
    gtsam::noiseModel::Diagonal::Variances(Vector6);
  graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
    key_frame_size - 1, min_id, pose_from.between(pose_to), optimize_noise));

  RCLCPP_INFO_STREAM(get_logger(), "optimized...");

  // update
  optimizer_->update(graph_);
  optimizer_->update();

  graph_.resize(0);

  is_loop_closed_ = true;
}

void GraphBasedSLAM::key_frame_callback(const lidar_graph_slam_msgs::msg::KeyFrame::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(key_frame_update_mutex_);
  if (!is_initialized_key_frame_) is_initialized_key_frame_ = true;

  auto key_frame_size = key_frame_array_.keyframes.size();
  auto latest_key_frame = geometry_pose_to_gtsam_pose(msg->pose);
  if (key_frame_array_.keyframes.empty()) {
    gtsam::Vector Vector6(6);
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, latest_key_frame, prior_noise_));
    initial_estimate_.insert(0, latest_key_frame);
  } else {
    auto previous_key_frame = geometry_pose_to_gtsam_pose(key_frame_array_.keyframes.back().pose);
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
      key_frame_size - 1, key_frame_size, previous_key_frame.between(latest_key_frame),
      prior_noise_));
    initial_estimate_.insert(key_frame_size, latest_key_frame);
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
  modified_key_frame_publisher_->publish(key_frame_array_);

  PointType key_frame_point;
  key_frame_point.x = key_frame.pose.position.x;
  key_frame_point.y = key_frame.pose.position.y;
  key_frame_point.z = key_frame.pose.position.z;
  key_frame_point_->points.emplace_back(key_frame_point);

  key_frame_raw_array_.keyframes.emplace_back(*msg);

  if (is_loop_closed_) {
    adjust_pose();
    is_loop_closed_ = false;
  }

  publish_map();
  update_estimate_path();
}

pcl::PointCloud<PointType>::Ptr GraphBasedSLAM::transform_point_cloud(
  const pcl::PointCloud<PointType>::Ptr input_cloud_ptr, const Eigen::Matrix4f transform_matrix)
{
  pcl::PointCloud<PointType>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::transformPointCloud(*input_cloud_ptr, *transform_cloud_ptr, transform_matrix);

  return transform_cloud_ptr;
}

void GraphBasedSLAM::adjust_pose()
{
  auto current_estimate = optimizer_->calculateEstimate();
  auto estimated_pose = current_estimate.at<gtsam::Pose3>(current_estimate.size() - 1);

  for (std::size_t idx = 0; idx < current_estimate.size(); idx++) {
    key_frame_array_.keyframes[idx].pose =
      gtsam_pose_to_geometry_pose(current_estimate.at<gtsam::Pose3>(idx));

    PointType key_frame_point;
    key_frame_point.x = key_frame_array_.keyframes[idx].pose.position.x;
    key_frame_point.y = key_frame_array_.keyframes[idx].pose.position.y;
    key_frame_point.z = key_frame_array_.keyframes[idx].pose.position.z;
    key_frame_point_->points[idx] = key_frame_point;
  }
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

void GraphBasedSLAM::publish_map()
{
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

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(GraphBasedSLAM)
