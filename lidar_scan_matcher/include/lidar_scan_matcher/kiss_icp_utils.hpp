#ifndef _LIDAR_SCAN_MATCHER__UTILS_HPP_
#define _LIDAR_SCAN_MATCHER__UTILS_HPP_

#include <Eigen/Core>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <vector>

#include "kiss_icp/pipeline/KissICP.hpp"

inline std::vector<Eigen::Vector3d> point_cloud2_to_eigen(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(msg->height * msg->width);
  sensor_msgs::PointCloud2ConstIterator<float> msg_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> msg_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> msg_z(*msg, "z");
  for (size_t i = 0; i < msg->height * msg->width; ++i, ++msg_x, ++msg_y, ++msg_z) {
    points.emplace_back(*msg_x, *msg_y, *msg_z);
  }
  return points;
}

inline auto normalize_timestamps(const std::vector<double> & timestamps)
{
  const auto [min_it, max_it] = std::minmax_element(timestamps.cbegin(), timestamps.cend());
  const double min_timestamp = *min_it;
  const double max_timestamp = *max_it;

  std::vector<double> timestamps_normalized(timestamps.size());
  std::transform(
    timestamps.cbegin(), timestamps.cend(), timestamps_normalized.begin(),
    [&](const auto & timestamp) {
      return (timestamp - min_timestamp) / (max_timestamp - min_timestamp);
    });
  return timestamps_normalized;
}

inline auto get_timestamp_field(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  sensor_msgs::msg::PointField timestamp_field;
  for (const auto & field : msg->fields) {
    if ((field.name == "t" || field.name == "timestamp" || field.name == "time")) {
      timestamp_field = field;
    }
  }
  if (!timestamp_field.count) {
    throw std::runtime_error("Field 't', 'timestamp', or 'time'  does not exist");
  }
  return timestamp_field;
}

inline auto extract_timestamps_from_msg(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
  const sensor_msgs::msg::PointField & field)
{
  auto extract_timestamps =
    [&msg]<typename T>(sensor_msgs::PointCloud2ConstIterator<T> && it) -> std::vector<double> {
    const size_t n_points = msg->height * msg->width;
    std::vector<double> timestamps;
    timestamps.reserve(n_points);
    for (size_t i = 0; i < n_points; ++i, ++it) {
      timestamps.emplace_back(static_cast<double>(*it));
    }
    return normalize_timestamps(timestamps);
  };

  // Get timestamp field that must be one of the following : {t, timestamp, time}
  auto timestamp_field = get_timestamp_field(msg);

  // According to the type of the timestamp == type, return a PointCloud2ConstIterator<type>
  using sensor_msgs::PointCloud2ConstIterator;
  if (timestamp_field.datatype == sensor_msgs::msg::PointField::UINT32) {
    return extract_timestamps(PointCloud2ConstIterator<uint32_t>(*msg, timestamp_field.name));
  } else if (timestamp_field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
    return extract_timestamps(PointCloud2ConstIterator<float>(*msg, timestamp_field.name));
  } else if (timestamp_field.datatype == sensor_msgs::msg::PointField::FLOAT64) {
    return extract_timestamps(PointCloud2ConstIterator<double>(*msg, timestamp_field.name));
  }

  // timestamp type not supported, please open an issue :)
  throw std::runtime_error("timestamp field type not supported");
}

inline std::vector<double> get_timestamps(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  auto timestamp_field = get_timestamp_field(msg);

  // Extract timestamps from cloud_msg
  std::vector<double> timestamps = extract_timestamps_from_msg(msg, timestamp_field);

  return timestamps;
}

#endif
