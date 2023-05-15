#ifndef LIDAR_GRAPH_SLAM__LIDAR_GRAPH_SLAM_UTILS_HPP_
#define LIDAR_GRAPH_SLAM__LIDAR_GRAPH_SLAM_UTILS_HPP_

#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <tf2/convert.h>

namespace lidar_graph_slam_utils
{

Eigen::Matrix4f geometry_pose_to_matrix(const geometry_msgs::msg::Pose pose)
{
  Eigen::Affine3d affine;
  tf2::fromMsg(pose, affine);
  Eigen::Matrix4f matrix = affine.matrix().cast<float>();
  return matrix;
}

gtsam::Pose3 geometry_pose_to_gtsam_pose(const geometry_msgs::msg::Pose pose)
{
  gtsam::Pose3 gtsam_pose = gtsam::Pose3(
    gtsam::Quaternion(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z),
    gtsam::Point3(pose.position.x, pose.position.y, pose.position.z));
  return gtsam_pose;
}

geometry_msgs::msg::Pose gtsam_pose_to_geometry_pose(const gtsam::Pose3 gtsam_pose)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = gtsam_pose.x();
  pose.position.y = gtsam_pose.y();
  pose.position.z = gtsam_pose.z();

  gtsam::Quaternion quaternion = gtsam_pose.rotation().toQuaternion();
  pose.orientation.w = quaternion.w();
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  return pose;
}

geometry_msgs::msg::Vector3 convert_quaternion_to_euler(
  const geometry_msgs::msg::Quaternion quaternion)
{
  geometry_msgs::msg::Vector3 euler;

  tf2::Quaternion quat(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
  tf2::Matrix3x3 mat(quat);
  mat.getRPY(euler.x, euler.y, euler.z);

  return euler;
}

geometry_msgs::msg::Pose convert_transform_to_pose(
  const geometry_msgs::msg::TransformStamped transform_stamped)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform_stamped.transform.translation.x;
  pose.position.y = transform_stamped.transform.translation.y;
  pose.position.z = transform_stamped.transform.translation.z;
  pose.orientation.w = transform_stamped.transform.rotation.w;
  pose.orientation.x = transform_stamped.transform.rotation.x;
  pose.orientation.y = transform_stamped.transform.rotation.y;
  pose.orientation.z = transform_stamped.transform.rotation.z;
  return pose;
}

Eigen::Matrix4f convert_pose_to_matrix(const geometry_msgs::msg::Pose pose)
{
  Eigen::Affine3d affine;
  tf2::fromMsg(pose, affine);
  Eigen::Matrix4f matrix = affine.matrix().cast<float>();
  return matrix;
}

geometry_msgs::msg::Pose convert_matrix_to_pose(const Eigen::Matrix4f matrix)
{
  geometry_msgs::msg::Pose pose;

  const Eigen::Vector3d position = matrix.block<3, 1>(0, 3).cast<double>();
  const Eigen::Quaterniond quaternion(matrix.block<3, 3>(0, 0).cast<double>());

  pose.position = tf2::toMsg(position);
  pose.orientation = tf2::toMsg(quaternion);

  return pose;
}

Eigen::Matrix4f convert_transform_to_matrix(
  const geometry_msgs::msg::TransformStamped transform_stamped)
{
  return convert_pose_to_matrix(convert_transform_to_pose(transform_stamped));
}

geometry_msgs::msg::Vector3 create_scale(const double x, const double y, const double z)
{
  geometry_msgs::msg::Vector3 vector3;
  vector3.x = x;
  vector3.y = y;
  vector3.z = z;
  return vector3;
}

std_msgs::msg::ColorRGBA create_color(
  const double a, const double r, const double g, const double b)
{
  std_msgs::msg::ColorRGBA color;
  color.a = a;
  color.r = r;
  color.g = g;
  color.b = b;
  return color;
}

visualization_msgs::msg::Marker create_marker(
  const geometry_msgs::msg::Pose pose, const rclcpp::Time stamp, const int32_t type, const int id,
  const std::string text, geometry_msgs::msg::Vector3 scale, std_msgs::msg::ColorRGBA color)
{
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.ns = "marker";
  marker.id = id;
  marker.type = type;
  marker.text = text;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = pose;
  marker.scale = scale;
  marker.color = color;

  return marker;
}
}  // namespace lidar_graph_slam_utils

#endif
