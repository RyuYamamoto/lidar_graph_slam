#ifndef LIDAR_GRAPH_SLAM__GRAPH_BASED_SLAM_HPP_
#define LIDAR_GRAPH_SLAM__GRAPH_BASED_SLAM_HPP_

#include "lidar_graph_slam_utils/lidar_graph_slam_utils.hpp"

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lidar_graph_slam_msgs/msg/key_frame.hpp"
#include "lidar_graph_slam_msgs/msg/key_frame_array.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclomp/ndt_omp.h>

class GraphBasedSLAM : public rclcpp::Node
{
public:
  GraphBasedSLAM(const rclcpp::NodeOptions & node_options);
  ~GraphBasedSLAM() = default;

  void key_frame_callback(const lidar_graph_slam_msgs::msg::KeyFrame::SharedPtr msg);
  void optimization_callback();

private:
  rclcpp::Subscription<lidar_graph_slam_msgs::msg::KeyFrame>::SharedPtr key_frame_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr modified_map_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  gtsam::NonlinearFactorGraph graph_;

  lidar_graph_slam_msgs::msg::KeyFrameArray key_frame_array_;

  std::mutex mutex_;

  bool is_initialized_key_frame_{false};
};

#endif
