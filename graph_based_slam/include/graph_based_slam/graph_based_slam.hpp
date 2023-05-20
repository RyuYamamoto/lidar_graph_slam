#ifndef LIDAR_GRAPH_SLAM__GRAPH_BASED_SLAM_HPP_
#define LIDAR_GRAPH_SLAM__GRAPH_BASED_SLAM_HPP_

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <lidar_graph_slam_utils/lib/kd_tree.hpp>
#include <lidar_graph_slam_utils/lidar_graph_slam_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>

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
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>

using PointType = pcl::PointXYZ;

class GraphBasedSLAM : public rclcpp::Node
{
public:
  GraphBasedSLAM(const rclcpp::NodeOptions & node_options);
  ~GraphBasedSLAM() = default;

  bool detect_loop(
    const lidar_graph_slam_msgs::msg::KeyFrame latest_key_frame,
    //const lidar_graph_slam_msgs::msg::KeyFrameArray key_frame_array,
    const pcl::PointCloud<PointType>::Ptr key_frame_cloud,
    pcl::PointCloud<PointType>::Ptr & nearest_key_frame_cloud, int & closest_key_frame_id);

  void key_frame_callback(const lidar_graph_slam_msgs::msg::KeyFrame::SharedPtr msg);
  void optimization_callback();

  void publish_candidate();

  void publish_map();

  void publish_key_frame_marker();

  pcl::PointCloud<PointType>::Ptr transform_point_cloud(
    const pcl::PointCloud<PointType>::Ptr input_cloud_ptr, const Eigen::Matrix4f transform_matrix);

  pcl::Registration<PointType, PointType>::Ptr get_registration(
    const std::string registration_method);

private:
  rclcpp::Subscription<lidar_graph_slam_msgs::msg::KeyFrame>::SharedPtr key_frame_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr modified_map_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    odometry_key_frame_marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    modified_key_frame_marker_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr candidate_key_frame_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  // registration
  pcl::Registration<PointType, PointType>::Ptr registration_;

  //std::shared_ptr<KDTree> kd_tree_;
  pcl::KdTreeFLANN<PointType>::Ptr kd_tree_;

  gtsam::NonlinearFactorGraph graph_;
  gtsam::ISAM2 optimizer_;
  gtsam::Values initial_estimate_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;

  pcl::PointCloud<PointType>::Ptr key_frame_point_;
  lidar_graph_slam_msgs::msg::KeyFrameArray key_frame_array_;
  lidar_graph_slam_msgs::msg::KeyFrameArray key_frame_raw_array_;

  std::mutex mutex_;

  bool is_initialized_key_frame_{false};
  int search_key_frame_num_;
  double score_threshold_;
  double search_radius_;

  nav_msgs::msg::Path candidate_line_;
};

#endif
