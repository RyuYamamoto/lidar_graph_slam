#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

using PointType = pcl::PointXYZI;

class PointsPreFiltering : public rclcpp::Node
{
public:
  PointsPreFiltering(const rclcpp::NodeOptions & node_options)
  : Node("points_prefiltering", node_options)
  {
    leaf_size_ = this->declare_parameter<double>("leaf_size");
    random_sample_num_ = this->declare_parameter<double>("random_sample_num");
    mean_k_ = this->declare_parameter<int>("mean_k");
    stddev_ = this->declare_parameter<double>("stddev");
    min_x_ = this->declare_parameter<double>("min_x");
    max_x_ = this->declare_parameter<double>("max_x");
    min_y_ = this->declare_parameter<double>("min_y");
    max_y_ = this->declare_parameter<double>("max_y");
    min_z_ = this->declare_parameter<double>("min_z");
    max_z_ = this->declare_parameter<double>("max_z");
    min_distance_cloud_ = this->declare_parameter<double>("min_distance_cloud");
    max_distance_cloud_ = this->declare_parameter<double>("max_distance_cloud");

    random_sample_filter_.reset(new pcl::RandomSample<PointType>);
    voxel_grid_filter_.reset(new pcl::VoxelGrid<PointType>);
    outlier_filter_.reset(new pcl::StatisticalOutlierRemoval<PointType>);

    sensor_points_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points_raw", rclcpp::SensorDataQoS().keep_last(5),
      std::bind(&PointsPreFiltering::callback_sensor_points, this, std::placeholders::_1));

    filter_points_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 5);
  }

  void callback_sensor_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<PointType>::Ptr input_points(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*msg, *input_points);

    pcl::PointCloud<PointType>::Ptr dist_filter_points(new pcl::PointCloud<PointType>);
    distance_filter(input_points, dist_filter_points);

    pcl::PointCloud<PointType>::Ptr crop_points(new pcl::PointCloud<PointType>);
    crop(dist_filter_points, crop_points);

    pcl::PointCloud<PointType>::Ptr voxel_filter_points(new pcl::PointCloud<PointType>);
    downsample(dist_filter_points, voxel_filter_points);

    pcl::PointCloud<PointType>::Ptr outlier_filter_points(new pcl::PointCloud<PointType>);
    outlier_filter(voxel_filter_points, outlier_filter_points);

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*outlier_filter_points, output_msg);
    output_msg.header = msg->header;

    filter_points_publisher_->publish(output_msg);
  }

  void crop(
    const pcl::PointCloud<PointType>::Ptr input_points_ptr,
    const pcl::PointCloud<PointType>::Ptr & output_points_ptr)
  {
    for (const auto & point : input_points_ptr->points) {
      if (
        (min_x_ < point.x and point.x < max_x_) and (min_y_ < point.y and point.y < max_y_) and
        (min_z_ < point.z and point.z < max_z_)) {
        output_points_ptr->points.emplace_back(point);
      }
    }
  }

  void distance_filter(
    const pcl::PointCloud<PointType>::Ptr input_points_ptr,
    const pcl::PointCloud<PointType>::Ptr & output_points_ptr)
  {
    for (const auto & point : input_points_ptr->points) {
      const double distance = point.getVector3fMap().norm();
      if (min_distance_cloud_ < distance) {
        output_points_ptr->points.emplace_back(point);
      }
    }
  }

  void downsample(
    const pcl::PointCloud<PointType>::Ptr input_points_ptr,
    const pcl::PointCloud<PointType>::Ptr & output_points_ptr)
  {
    voxel_grid_filter_->setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    voxel_grid_filter_->setInputCloud(input_points_ptr);
    voxel_grid_filter_->filter(*output_points_ptr);
  }

  void random_sampling(
    const pcl::PointCloud<PointType>::Ptr input_points_ptr,
    const pcl::PointCloud<PointType>::Ptr & output_points_ptr)
  {
    random_sample_filter_->setSample(random_sample_num_);
    random_sample_filter_->setInputCloud(input_points_ptr);
    random_sample_filter_->filter(*output_points_ptr);
  }

  void outlier_filter(
    const pcl::PointCloud<PointType>::Ptr input_points_ptr,
    const pcl::PointCloud<PointType>::Ptr & output_points_ptr)
  {
    outlier_filter_->setInputCloud(input_points_ptr);
    outlier_filter_->setMeanK(mean_k_);
    outlier_filter_->setStddevMulThresh(stddev_);
    outlier_filter_->filter(*output_points_ptr);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filter_points_publisher_;

  pcl::RandomSample<PointType>::Ptr random_sample_filter_;
  pcl::VoxelGrid<PointType>::Ptr voxel_grid_filter_;
  pcl::StatisticalOutlierRemoval<PointType>::Ptr outlier_filter_;

  double leaf_size_;
  double random_sample_num_;

  double min_distance_cloud_;
  double max_distance_cloud_;

  int mean_k_;
  double stddev_;

  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
  double min_z_;
  double max_z_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PointsPreFiltering)
