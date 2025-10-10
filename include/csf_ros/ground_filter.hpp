#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <csf_pcl/cloth_simulation_filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <string>
#include <vector>

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
  using PCLPoint = pcl::PointXYZI;
  using PCLPointCloud = pcl::PointCloud<PCLPoint>;

  void pointsCallback(sensor_msgs::msg::PointCloud2::UniquePtr msg);

  void initializeParameters();

  std::vector<double> crop_range_min_;
  std::vector<double> crop_range_max_;

  csf_pcl::ClothSimulationFilter<PCLPoint> csf_{true};

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
  std::vector<rclcpp::ParameterCallbackHandle::SharedPtr> parameter_callback_handles_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr off_ground_points_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
};
}  // namespace csf_ros
