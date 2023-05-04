#include "graph_based_slam/graph_based_slam.hpp"

GraphBasedSLAM::GraphBasedSLAM(const rclcpp::NodeOptions & node_options)
: Node("graph_based_slam", node_options)
{
}

void GraphBasedSLAM::optimization_callback() {}

void GraphBasedSLAM::key_frame_callback(
  const lidar_graph_slam_msgs::msg::KeyFrameArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  key_frame_array_ = *msg;
}
