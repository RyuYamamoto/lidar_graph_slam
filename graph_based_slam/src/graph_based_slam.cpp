#include "graph_based_slam/graph_based_slam.hpp"

using namespace lidar_graph_slam_utils;

GraphBasedSLAM::GraphBasedSLAM(const rclcpp::NodeOptions & node_options)
: Node("graph_based_slam", node_options)
{
  search_radius_ = this->declare_parameter<double>("search_radius");
  score_threshold_ = this->declare_parameter<double>("score_threshold");
  search_key_frame_num_ = this->declare_parameter<int>("search_key_frame_num");

  key_frame_subscriber_ = this->create_subscription<lidar_graph_slam_msgs::msg::KeyFrame>(
    "key_frame", 5, std::bind(&GraphBasedSLAM::key_frame_callback, this, std::placeholders::_1));

  candidate_key_frame_publisher_ =
    this->create_publisher<nav_msgs::msg::Path>("candidate_key_frame", 5);
  odometry_key_frame_marker_publisher_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("odometry_key_frame_marker", 5);
  modified_key_frame_marker_publisher_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("modified_key_frame_marker", 5);
  modified_map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "modified_map", rclcpp::QoS{1}.transient_local());

  // kd_tree_ = std::make_shared<KDTree>();
  kd_tree_.reset(new pcl::KdTreeFLANN<PointType>());
  key_frame_point_.reset(new pcl::PointCloud<PointType>);

  const std::string registration_method =
    this->declare_parameter<std::string>("registration_method");
  registration_ = get_registration(registration_method);

  prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6).finished());

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

    registration = ndt_omp;
  } else if (registration_method == "GICP") {
    pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>::Ptr gicp(
      new pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>());

    const double correspondence_distance =
      this->declare_parameter<double>("correspondence_distance");
    const double max_iteration = this->declare_parameter<int>("max_iteration");
    const double transformation_epsilon = this->declare_parameter<double>("transformation_epsilon");
    const double euclidean_fitness_epsilon =
      this->declare_parameter<double>("euclidean_fitness_epsilon");
    const double ransac_iteration = this->declare_parameter<double>("ransac_iteration");

    gicp->setMaxCorrespondenceDistance(correspondence_distance);
    gicp->setMaximumIterations(max_iteration);
    gicp->setTransformationEpsilon(transformation_epsilon);
    gicp->setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    gicp->setRANSACIterations(ransac_iteration);

    registration = gicp;
  }

  return registration;
}

bool GraphBasedSLAM::detect_loop(
  const lidar_graph_slam_msgs::msg::KeyFrame latest_key_frame,
  // const lidar_graph_slam_msgs::msg::KeyFrameArray key_frame_array,
  const pcl::PointCloud<PointType>::Ptr key_frame_cloud,
  pcl::PointCloud<PointType>::Ptr & nearest_key_frame_cloud, int & closest_key_frame_id)
{
  rclcpp::Time latest_stamp = latest_key_frame.header.stamp;
  geometry_msgs::msg::Pose latest_pose = latest_key_frame.pose;

  std::lock_guard<std::mutex> lock(mutex_);

  // kd_tree_->set_input_cloud(key_frame_array);
  kd_tree_->setInputCloud(key_frame_cloud);

  std::vector<int> indices;
  std::vector<float> dists;
  PointType latest_key_point;
  latest_key_point.x = latest_pose.position.x;
  latest_key_point.y = latest_pose.position.y;
  latest_key_point.z = latest_pose.position.z;
  // kd_tree_->radius_search(latest_pose, search_radius_, indices);
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

  const int key_frame_size = key_frame_array_.keyframes.size();
  const auto latest_key_frame = key_frame_array_.keyframes.back();
  pcl::PointCloud<PointType>::Ptr nearest_key_frame_cloud(new pcl::PointCloud<PointType>);

  int closest_key_frame_id;
  if (!detect_loop(
        latest_key_frame, key_frame_point_, nearest_key_frame_cloud, closest_key_frame_id)) {
    return;
  }

  // for visualize loop detection
  geometry_msgs::msg::PoseStamped pose_to;
  pose_to.pose = latest_key_frame.pose;
  pose_to.header = latest_key_frame.header;
  geometry_msgs::msg::PoseStamped pose_from;
  pose_from.pose = key_frame_array_.keyframes[closest_key_frame_id].pose;
  pose_from.header = key_frame_array_.keyframes[closest_key_frame_id].header;
  candidate_line_.poses.emplace_back(pose_from);
  candidate_line_.poses.emplace_back(pose_to);
  candidate_line_.header.frame_id = "map";
  candidate_line_.header.stamp = latest_key_frame.header.stamp;

  candidate_key_frame_publisher_->publish(candidate_line_);

  pcl::PointCloud<PointType>::Ptr latest_key_frame_cloud(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(latest_key_frame.cloud, *latest_key_frame_cloud);
  registration_->setInputSource(latest_key_frame_cloud);
  registration_->setInputTarget(nearest_key_frame_cloud);

  Eigen::Matrix4f initial_pose = convert_pose_to_matrix(latest_key_frame.pose);

  pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>);
  registration_->align(*output_cloud);

  const Eigen::Matrix4f transform = registration_->getFinalTransformation();
  const double fitness_score = registration_->getFitnessScore();
  const bool has_converged = registration_->hasConverged();

  if (!has_converged or score_threshold_ < fitness_score) {
    return;
  }

  RCLCPP_INFO_STREAM(get_logger(), "fitness_score : " << fitness_score);
  if (fitness_score < score_threshold_) {
    RCLCPP_INFO_STREAM(get_logger(), "update pose graph.");

    std::lock_guard<std::mutex> lock(mutex_);
    // correct position
    auto gtsam_pose_from = geometry_pose_to_gtsam_pose(convert_matrix_to_pose(transform));
    // candidate position
    auto gtsam_pose_to = geometry_pose_to_gtsam_pose(pose_to.pose);
    gtsam::noiseModel::Diagonal::shared_ptr optimize_noise = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << fitness_score, fitness_score, fitness_score, fitness_score,
       fitness_score, fitness_score)
        .finished());
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
      key_frame_size - 1, closest_key_frame_id, gtsam_pose_from.between(gtsam_pose_to),
      optimize_noise));

    // update
    optimizer_.update(graph_);
    optimizer_.update();

    graph_.resize(0);

    auto current_estimate = optimizer_.calculateEstimate();
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

    publish_map();
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
  key_frame.header = msg->header;
  key_frame.cloud = msg->cloud;
  key_frame.pose = gtsam_pose_to_geometry_pose(estimated_pose);
  key_frame_array_.keyframes.emplace_back(key_frame);
  PointType key_frame_point;
  key_frame_point.x = key_frame.pose.position.x;
  key_frame_point.y = key_frame.pose.position.y;
  key_frame_point.z = key_frame.pose.position.z;
  key_frame_point_->points.emplace_back(key_frame_point);

  for (std::size_t idx = 0; idx < current_estimate.size(); idx++) {
    key_frame_array_.keyframes[idx].pose =
      gtsam_pose_to_geometry_pose(current_estimate.at<gtsam::Pose3>(idx));
  }

  key_frame_raw_array_.keyframes.emplace_back(*msg);

  publish_map();

  publish_key_frame_marker();
}

pcl::PointCloud<PointType>::Ptr GraphBasedSLAM::transform_point_cloud(
  const pcl::PointCloud<PointType>::Ptr input_cloud_ptr, const Eigen::Matrix4f transform_matrix)
{
  pcl::PointCloud<PointType>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::transformPointCloud(*input_cloud_ptr, *transform_cloud_ptr, transform_matrix);

  return transform_cloud_ptr;
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
