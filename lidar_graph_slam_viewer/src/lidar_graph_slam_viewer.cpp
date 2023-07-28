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

#include "lidar_graph_slam_viewer/lidar_graph_slam_viewer.hpp"

LidarGraphSlamViewer::LidarGraphSlamViewer(const rclcpp::NodeOptions & node_options)
: Node("lidar_graph_slam_viewer", node_options)
{
  map_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_map", rclcpp::QoS(1).transient_local(),
    std::bind(&LidarGraphSlamViewer::map_callback, this, std::placeholders::_1));
  sensor_points_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&LidarGraphSlamViewer::sensor_points_callback, this, std::placeholders::_1));
  key_frame_array_subscriber_ = create_subscription<lidar_graph_slam_msgs::msg::KeyFrameArray>(
    "modified_key_frame", 5,
    std::bind(&LidarGraphSlamViewer::key_frame_callback, this, std::placeholders::_1));
  candidate_path_subscriber_ = create_subscription<nav_msgs::msg::Path>(
    "candidate_key_frame", 5,
    std::bind(&LidarGraphSlamViewer::candidate_path_callback, this, std::placeholders::_1));

  toggle_client_ =
    create_client<rosbag2_interfaces::srv::TogglePaused>("rosbag2_player/toggle_paused");
  toggle_client_->wait_for_service(std::chrono::milliseconds(200));

  rate_client_ = create_client<rosbag2_interfaces::srv::SetRate>("rosbag2_player/set_rate");
  rate_client_->wait_for_service(std::chrono::milliseconds(200));

  latest_sensor_points_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  create_viewer();
}

LidarGraphSlamViewer::~LidarGraphSlamViewer() { viewer_thread_.join(); }

void LidarGraphSlamViewer::sensor_points_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto map_to_base_frame = get_transform("base_link", "map");
  const Eigen::Affine3d frame_affine = tf2::transformToEigen(map_to_base_frame);
  latest_pose_matrix_ = frame_affine.matrix().cast<float>();

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *input_cloud_ptr);
  transform_cloud_ptr = transform_point_cloud(input_cloud_ptr, map_to_base_frame);
  latest_sensor_points_ = transform_cloud_ptr;
}

void LidarGraphSlamViewer::map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (map_ == nullptr) {
    map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_raw(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *map_);

  is_map_loaded_ = true;
  is_first_map_plot_ = true;
}

void LidarGraphSlamViewer::key_frame_callback(
  const lidar_graph_slam_msgs::msg::KeyFrameArray::SharedPtr msg)
{
  key_frame_ = *msg;
  is_key_frame_plot_ = true;
}

void LidarGraphSlamViewer::candidate_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  candidate_path_ = *msg;
  is_candidate_key_frame_plot_ = true;
}

geometry_msgs::msg::TransformStamped LidarGraphSlamViewer::get_transform(
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

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarGraphSlamViewer::transform_point_cloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
  const geometry_msgs::msg::TransformStamped transform)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  const Eigen::Affine3d frame_affine = tf2::transformToEigen(transform);
  const Eigen::Matrix4f frame_matrix = frame_affine.matrix().cast<float>();
  pcl::transformPointCloud(*input_cloud_ptr, *transform_cloud_ptr, frame_matrix);

  return transform_cloud_ptr;
}

void LidarGraphSlamViewer::create_viewer()
{
  RCLCPP_INFO_STREAM(get_logger(), "create viewer...");
  std::this_thread::sleep_for(std::chrono::seconds(2));
  viewer_thread_ = std::thread(std::bind(&LidarGraphSlamViewer::viewer_thread, this));
  std::this_thread::sleep_for(std::chrono::seconds(2));
}

void LidarGraphSlamViewer::viewer_thread()
{
  auto viewer = guik::LightViewer::instance();

  viewer->register_ui_callback("rosbag player", [&]() {
    ImGui::Begin("rosbag player", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    if (ImGui::Button("start/pause")) {
      auto future = toggle_client_->async_send_request(
        std::make_shared<rosbag2_interfaces::srv::TogglePaused::Request>());
      future.wait_for(std::chrono::milliseconds(200));
    }

    if (ImGui::InputFloat("play rate", &play_rate_, 0.1f, 1.0f, "%.1f", 1)) {
      play_rate_ = std::max(0.1f, play_rate_);
      auto request = std::make_shared<rosbag2_interfaces::srv::SetRate::Request>();
      request->rate = play_rate_;
      auto future = rate_client_->async_send_request(request);
      future.wait_for(std::chrono::milliseconds(200));
    }
  });

  while (viewer->spin_once()) {
    if (!is_map_loaded_) {
      continue;
    }

    if (is_first_map_plot_) {
      auto map_cloud_buffer = glk::create_point_cloud_buffer(*map_);

      viewer->update_drawable("point_cloud_map", map_cloud_buffer, guik::Rainbow());

      is_first_map_plot_ = false;
    }

    if (is_key_frame_plot_) {
      key_frame_line_.clear();
      int idx = 0;
      for (auto key_frame : key_frame_.keyframes) {
        Eigen::Vector3f vertices;
        vertices.x() = key_frame.pose.position.x;
        vertices.y() = key_frame.pose.position.y;
        vertices.z() = key_frame.pose.position.z;
        key_frame_line_.emplace_back(vertices);

        Eigen::Affine3d affine;
        tf2::fromMsg(key_frame.pose, affine);
        viewer->update_drawable(
          fmt::format("key_frame_{}", idx++), glk::Primitives::coordinate_system(),
          guik::VertexColor(affine.matrix().cast<float>()));
      }
      auto line = std::make_shared<glk::ThinLines>(key_frame_line_, true);

      line->set_line_width(2.0f);
      viewer->update_drawable("key_frame_line", line, guik::FlatColor(0.0f, 1.0f, 0.0f, 1.0f));

      is_key_frame_plot_ = false;
    }

    if (is_candidate_key_frame_plot_) {
      std::vector<Eigen::Vector3f> candidate_line;
      for (auto candidate : candidate_path_.poses) {
        Eigen::Vector3f vertices;
        vertices.x() = candidate.pose.position.x;
        vertices.y() = candidate.pose.position.y;
        vertices.z() = candidate.pose.position.z;
        candidate_line.emplace_back(vertices);
      }

      auto line = std::make_shared<glk::ThinLines>(candidate_line, true);

      line->set_line_width(1.0f);
      viewer->update_drawable("candidate_line", line, guik::FlatColor(1.0f, 1.0f, 1.0f, 0.5f));
      is_candidate_key_frame_plot_ = false;
    }

    if (latest_sensor_points_ == nullptr) continue;
    if (latest_sensor_points_->points.size() == 0) continue;
    auto cloud_buffer = glk::create_point_cloud_buffer(*latest_sensor_points_);
    viewer->update_drawable("sensor", cloud_buffer, guik::FlatColor(1.0, 1.0, 1.0, 0.2));

    viewer->update_drawable(
      "base_link", glk::Primitives::coordinate_system(),
      guik::VertexColor(latest_pose_matrix_.matrix().cast<float>()));
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(LidarGraphSlamViewer)
