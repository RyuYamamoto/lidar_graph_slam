#include "graph_based_slam/graph_based_slam.hpp"

GraphBasedSLAM::GraphBasedSLAM(const rclcpp::NodeOptions & node_options)
: Node("graph_based_slam", node_options)
{
  const double rate = declare_parameter<double>("rate");
  timer_ = create_timer(
    this, get_clock(), rclcpp::Rate(rate).period(),
    std::bind(&GraphBasedSLAM::optimization_callback, this));
}

void GraphBasedSLAM::optimization_callback()
{
  if (!is_initialized_key_frame_) return;
}

void GraphBasedSLAM::key_frame_callback(
  const lidar_graph_slam_msgs::msg::KeyFrameArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  key_frame_array_ = *msg;
  is_initialized_key_frame_ = true;
}
