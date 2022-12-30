#include <ndt_slam/ndt_slam.h>
#include <ndt_slam/ndt_slam_utils.h>

NDTSlam::NDTSlam() : Node("ndt_slam")
{
  base_frame_id_ = declare_parameter("base_frame_id", "base_link");
  min_scan_range_ = declare_parameter("min_scan_range", 5.0);
  max_scan_range_ = declare_parameter("max_scan_range", 200.0);
  min_add_scan_shift_ = declare_parameter("min_add_scan_shift", 1.0);
  ndt_res_ = declare_parameter("ndt_res", 1.0);
  trans_eps_ = declare_parameter("trans_eps", 0.01);
  step_size_ = declare_parameter("step_size", 0.1);
  leaf_size_ = declare_parameter("leaf_size", 2.0);
  max_iter_ = declare_parameter("max_iter", 30);
  omp_num_thread_ = declare_parameter("omp_num_thread", 0);
  use_imu_ = declare_parameter("use_imu", false);

  ndt_.setTransformationEpsilon(trans_eps_);
  ndt_.setStepSize(step_size_);
  ndt_.setResolution(ndt_res_);
  ndt_.setMaximumIterations(max_iter_);
  if (0 < omp_num_thread_) ndt_.setNumThreads(omp_num_thread_);
  ndt_.setNeighborhoodSearchMethod(pclomp::KDTREE);

  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // create subscriber
  points_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&NDTSlam::pointsCallback, this, std::placeholders::_1));

  // create publisher
  ndt_aligned_cloud_publisher_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("alinged_cloud", 1000);
  ndt_map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ndt_map", 1000);
  ndt_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ndt_pose", 1000);
  transform_probability_publisher_ =
    this->create_publisher<std_msgs::msg::Float32>("transform_probability", 1);

  // create service server
  save_map_service_ = this->create_service<ndt_slam_srvs::srv::SaveMap>(
    "save_map",
    std::bind(&NDTSlam::saveMapService, this, std::placeholders::_1, std::placeholders::_2));
}

void NDTSlam::imuCorrect(Eigen::Matrix4f & pose, const rclcpp::Time stamp)
{
  static rclcpp::Time prev_stamp = stamp;
  static sensor_msgs::msg::Imu prev_imu = imu_;
  const double sampling_time = (stamp - prev_stamp).seconds();

  Eigen::Vector3f diff_rot;
  diff_rot.x() = (imu_.angular_velocity.x - prev_imu.angular_velocity.x) * sampling_time;
  diff_rot.y() = (imu_.angular_velocity.y - prev_imu.angular_velocity.y) * sampling_time;
  diff_rot.z() = (imu_.angular_velocity.z - prev_imu.angular_velocity.z) * sampling_time;

  imu_rotate_vec_ += diff_rot;

  Pose pose_vec = ndt_slam_utils::convertMatrixToPoseVec(pose);

  pose_vec.roll += imu_rotate_vec_.x();
  pose_vec.pitch += imu_rotate_vec_.y();
  pose_vec.yaw += imu_rotate_vec_.z();

  pose = ndt_slam_utils::convertPoseVecToMatrix(pose_vec);

  prev_imu = imu_;
  prev_stamp = stamp;
}

Pose NDTSlam::getCurrentPose() { return ndt_slam_utils::convertMatrixToPoseVec(pose_); }

void NDTSlam::limitCloudScanData(
  const pcl::PointCloud<PointType>::Ptr input_ptr,
  const pcl::PointCloud<PointType>::Ptr & output_ptr, const double min_scan_range,
  const double max_scan_range)
{
  for (auto point : input_ptr->points) {
    const double range = std::hypot(point.x, point.y);
    if (min_scan_range < range && range < max_scan_range) {
      output_ptr->push_back(point);
    }
  }
}

void NDTSlam::downsample(
  const pcl::PointCloud<PointType>::Ptr input_ptr,
  const pcl::PointCloud<PointType>::Ptr & output_ptr)
{
  pcl::VoxelGrid<PointType> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  voxel_grid_filter.setInputCloud(input_ptr);
  voxel_grid_filter.filter(*output_ptr);
}

void NDTSlam::pointsCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_points_ptr_msg)
{
  pcl::PointCloud<PointType>::Ptr points_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr limit_points_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr filtered_scan_ptr(new pcl::PointCloud<PointType>);

  const rclcpp::Time current_scan_time = input_points_ptr_msg->header.stamp;
  const std::string sensor_frame_id = input_points_ptr_msg->header.frame_id;
  pcl::fromROSMsg(*input_points_ptr_msg, *points_ptr);

  limitCloudScanData(points_ptr, limit_points_ptr, min_scan_range_, max_scan_range_);

  if (!map_) {
    pcl::PointCloud<PointType>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointType>);
    transformPointCloud(limit_points_ptr, transform_cloud_ptr, base_frame_id_, sensor_frame_id);
    map_.reset(new pcl::PointCloud<PointType>);
    map_->header.frame_id = "map";
    *map_ += *transform_cloud_ptr;
    ndt_.setInputTarget(map_);
  }

  pcl::PointCloud<PointType>::Ptr sensor_transform_cloud(new pcl::PointCloud<PointType>);
  transformPointCloud(limit_points_ptr, sensor_transform_cloud, base_frame_id_, sensor_frame_id);
  downsample(sensor_transform_cloud, filtered_scan_ptr);
  ndt_.setInputSource(filtered_scan_ptr);

  if (use_imu_) imuCorrect(pose_, current_scan_time);

  pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>);
  ndt_.align(*output_cloud, pose_);

  const bool convergenced = ndt_.hasConverged();
  const double fitness_score = ndt_.getFitnessScore();
  const int final_iterations = ndt_.getFinalNumIteration();

  if (!convergenced) RCLCPP_WARN(get_logger(), "NDT has not Convergenced!");

  pose_ = ndt_.getFinalTransformation();

  // publish tf
  ndt_pose_ = getCurrentPose();  // convert matrix to vec
  ndt_slam_utils::publishTF(broadcaster_, ndt_pose_, current_scan_time, "map", base_frame_id_);

  pcl::PointCloud<PointType>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::transformPointCloud(
    *limit_points_ptr, *transform_cloud_ptr,
    pose_ * ndt_slam_utils::convertGeometryTransformToMatrix(
              getTransform(base_frame_id_, sensor_frame_id)));

  previous_scan_time_ = current_scan_time;

  const double delta = std::hypot(ndt_pose_.x - previous_pose_.x, ndt_pose_.y - previous_pose_.y);
  if (min_add_scan_shift_ <= delta) {
    previous_pose_ = ndt_pose_;

    *map_ += *transform_cloud_ptr;
    ndt_.setInputTarget(map_);

    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*map_, map_msg);
    ndt_map_publisher_->publish(map_msg);
  }

  sensor_msgs::msg::PointCloud2 aligned_cloud_msg;
  pcl::toROSMsg(*output_cloud, aligned_cloud_msg);
  aligned_cloud_msg.header.stamp = current_scan_time;
  aligned_cloud_msg.header.frame_id = sensor_frame_id;
  ndt_aligned_cloud_publisher_->publish(aligned_cloud_msg);

  std_msgs::msg::Float32 transform_probability;
  transform_probability.data = ndt_.getTransformationProbability();
  transform_probability_publisher_->publish(transform_probability);

  geometry_msgs::msg::PoseStamped ndt_pose_msg;
  ndt_pose_msg.header.frame_id = "map";
  ndt_pose_msg.header.stamp = current_scan_time;
  ndt_pose_msg.pose = ndt_slam_utils::convertToGeometryPose(ndt_pose_);
  ndt_pose_publisher_->publish(ndt_pose_msg);

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "NDT has converged: " << convergenced << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "Number of iteration: " << final_iterations << std::endl;
  std::cout << "delta: " << delta << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

void NDTSlam::transformPointCloud(
  pcl::PointCloud<PointType>::Ptr input_ptr, pcl::PointCloud<PointType>::Ptr & output_ptr,
  const std::string target_frame, const std::string source_frame)
{
  geometry_msgs::msg::TransformStamped sensor_frame_transform =
    getTransform(target_frame, source_frame);
  const Eigen::Affine3d base_to_sensor_frame_affine = tf2::transformToEigen(sensor_frame_transform);
  const Eigen::Matrix4f base_to_sensor_frame_matrix =
    base_to_sensor_frame_affine.matrix().cast<float>();
  pcl::transformPointCloud(*input_ptr, *output_ptr, base_to_sensor_frame_matrix);
}

geometry_msgs::msg::TransformStamped NDTSlam::getTransform(
  const std::string target_frame, const std::string source_frame)
{
  geometry_msgs::msg::TransformStamped frame_transform;
  try {
    frame_transform = tf_buffer_.lookupTransform(
      target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    frame_transform.header.stamp = rclcpp::Clock().now();
    frame_transform.header.frame_id = target_frame;
    frame_transform.child_frame_id = source_frame;
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

bool NDTSlam::saveMapService(
  const ndt_slam_srvs::srv::SaveMap::Request::SharedPtr req,
  ndt_slam_srvs::srv::SaveMap::Response::SharedPtr res)
{
  pcl::PointCloud<PointType>::Ptr map_cloud(new pcl::PointCloud<PointType>);

  if (req->resolution <= 0.0) {
    map_cloud = map_;
  } else {
    pcl::VoxelGrid<PointType> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(req->resolution, req->resolution, req->resolution);
    voxel_grid_filter.setInputCloud(map_);
    voxel_grid_filter.filter(*map_cloud);
  }

  map_cloud->header.frame_id = "map";
  int ret = pcl::io::savePCDFile(req->path, *map_cloud);
  res->ret = (ret == 0);

  return true;
}

void NDTSlam::odomCallback(const nav_msgs::msg::Odometry::SharedPtr & msg) { odom_ = *msg; }

void NDTSlam::imuCallback(const sensor_msgs::msg::Imu::SharedPtr & msg) { imu_ = *msg; }
