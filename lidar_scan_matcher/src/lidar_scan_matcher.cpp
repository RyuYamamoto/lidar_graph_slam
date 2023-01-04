#include "lidar_scan_matcher/lidar_scan_matcher.hpp"

LidarScanMatcher::LidarScanMatcher() : Node("lidar_scan_matcher_node")
{

}

void LidarScanMatcher::callback_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<PointType>::Ptr input_cloud_ptr(new pcl::PointCloud<PointType>);
}
