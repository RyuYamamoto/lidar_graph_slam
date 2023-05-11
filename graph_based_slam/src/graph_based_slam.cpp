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

  std::size_t key_frame_size = key_frame_array_.keyframes.size();
  auto latest_key_frame = key_frame_array_.keyframes.back();

  if(key_frame_size == 1) {
    // hoge
  } else {
    // fuga
  }
}

void GraphBasedSLAM::key_frame_callback(const lidar_graph_slam_msgs::msg::KeyFrame::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  key_frame_array_.keyframes.emplace_back(*msg);
  if (!is_initialized_key_frame_) is_initialized_key_frame_ = true;
}
