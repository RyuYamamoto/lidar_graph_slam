#ifndef _UTILS_HPP_
#define _UTILS_HPP_

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/convert.h>

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

Eigen::Matrix4f convert_transform_to_matrix(
  const geometry_msgs::msg::TransformStamped transform_stamped)
{
  return convert_pose_to_matrix(convert_transform_to_pose(transform_stamped));
}

#endif