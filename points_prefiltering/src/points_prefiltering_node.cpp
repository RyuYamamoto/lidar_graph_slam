#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>

class PointsPreFiltering : public rclcpp::Node
{
public:
  PointsPreFiltering(const rclcpp::NodeOptions & node_options)
  : Node("points_prefiltering", node_options)
  {}

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filter_points_publisher_;
};


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PointsPreFiltering)
