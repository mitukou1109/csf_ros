#include "csf_ros/ground_filter.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace csf_ros
{
GroundFilter::GroundFilter(const rclcpp::NodeOptions & options)
: GroundFilter("ground_filter", "", options)
{
}

GroundFilter::GroundFilter(
  const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & options)
: Node(node_name, ns, options)
{
  crop_range_min_ = declare_parameter<std::vector<double>>("crop_range.min", std::vector<double>{});
  crop_range_max_ = declare_parameter<std::vector<double>>("crop_range.max", std::vector<double>{});
  debug_ = declare_parameter<bool>("debug");

  csf_.params.bSloopSmooth = declare_parameter<bool>("enable_post_processing");
  csf_.params.class_threshold = declare_parameter<double>("class_threshold");
  csf_.params.cloth_resolution = declare_parameter<double>("cloth_resolution");
  csf_.params.interations = declare_parameter<int>("max_iterations");
  csf_.params.rigidness = declare_parameter<int>("rigidness");
  csf_.params.time_step = declare_parameter<double>("time_step");

  ground_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/ground_points", 1);

  off_ground_points_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("~/off_ground_points", 1);

  points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points", 1, std::bind(&GroundFilter::pointsCallback, this, std::placeholders::_1));
}

void GroundFilter::pointsCallback(sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
  const auto input_points = std::make_shared<PCLPointCloud>();
  pcl::fromROSMsg(*msg, *input_points);

  if (crop_range_min_.size() == 3 && crop_range_max_.size() == 3) {
    pcl::CropBox<PCLPoint> crop_box;
    crop_box.setInputCloud(input_points);
    crop_box.setMin(
      Eigen::Vector4f{
        static_cast<float>(crop_range_min_[0]), static_cast<float>(crop_range_min_[1]),
        static_cast<float>(crop_range_min_[2]), 1.0f});
    crop_box.setMax(
      Eigen::Vector4f{
        static_cast<float>(crop_range_max_[0]), static_cast<float>(crop_range_max_[1]),
        static_cast<float>(crop_range_max_[2]), 1.0f});
    crop_box.filter(*input_points);
  }

  std::vector<csf::Point> csf_points(input_points->size());
  std::transform(
    input_points->begin(), input_points->end(), csf_points.begin(),
    [](const PCLPoint & point) { return csf::Point{point.x, point.y, point.z}; });
  csf_.setPointCloud(csf_points);

  auto ground_indices = std::make_shared<pcl::Indices>();
  auto off_ground_indices = std::make_shared<pcl::Indices>();
  if (!debug_) {
    std::cout.setstate(std::ios_base::failbit);
  }
  csf_.do_filtering(*ground_indices, *off_ground_indices, false);
  if (!debug_) {
    std::cout.clear();
  }

  pcl::ExtractIndices<PCLPoint> extract_indices;
  extract_indices.setInputCloud(input_points);

  auto ground_points = std::make_shared<PCLPointCloud>();
  extract_indices.setIndices(ground_indices);
  extract_indices.filter(*ground_points);

  auto off_ground_points = std::make_shared<PCLPointCloud>();
  extract_indices.setIndices(off_ground_indices);
  extract_indices.filter(*off_ground_points);

  auto ground_points_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*ground_points, *ground_points_msg);
  ground_points_msg->header = msg->header;
  ground_points_pub_->publish(std::move(ground_points_msg));

  auto off_ground_points_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*off_ground_points, *off_ground_points_msg);
  off_ground_points_msg->header = msg->header;
  off_ground_points_pub_->publish(std::move(off_ground_points_msg));
}
}  // namespace csf_ros

RCLCPP_COMPONENTS_REGISTER_NODE(csf_ros::GroundFilter)
