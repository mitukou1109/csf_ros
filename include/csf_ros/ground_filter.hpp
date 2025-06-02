#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <CSF.h>
#include <pcl/point_cloud.h>

#include <string>

namespace csf_ros
{
class GroundFilter : public rclcpp::Node
{
public:
  explicit GroundFilter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  GroundFilter(
    const std::string & node_name, const std::string & ns,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void pointsCallback(const sensor_msgs::msg::PointCloud2::UniquePtr msg);

  CSF csf_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr off_ground_points_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
};
}  // namespace csf_ros
