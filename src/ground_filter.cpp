#include "csf_ros/ground_filter.hpp"

#include <pcl_ros/transforms.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <memory>
#include <string>
#include <utility>
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
  initializeParameters();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  ground_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/ground_points", 1);

  off_ground_points_pub_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/off_ground_points", 1);

  points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "points", 1, std::bind(&GroundFilter::pointsCallback, this, std::placeholders::_1));
}

void GroundFilter::pointsCallback(sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
  if (msg->data.empty()) {
    RCLCPP_WARN(get_logger(), "Received empty point cloud");
    return;
  }

  const auto input_points = std::make_shared<PCLPointCloud>();
  pcl::fromROSMsg(*msg, *input_points);

  if (gravity_aligned_frame_ != "") {
    try {
      pcl_ros::transformPointCloud(
        gravity_aligned_frame_, *input_points, *input_points, *tf_buffer_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), e.what());
      return;
    }
  }

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

  if (input_points->empty()) {
    RCLCPP_WARN(get_logger(), "Point cloud is empty after crop box filtering");
    return;
  }

  csf_.setInputCloud(input_points);
  auto ground_indices = std::make_shared<pcl::Indices>();
  csf_.filter(*ground_indices);

  pcl::ExtractIndices<PCLPoint> extract_indices;
  extract_indices.setInputCloud(input_points);

  auto ground_points = std::make_shared<PCLPointCloud>();
  extract_indices.setIndices(ground_indices);
  extract_indices.filter(*ground_points);

  auto off_ground_points = std::make_shared<PCLPointCloud>();
  extract_indices.setIndices(csf_.getRemovedIndices());
  extract_indices.filter(*off_ground_points);

  auto ground_points_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*ground_points, *ground_points_msg);
  ground_points_msg->header = msg->header;
  if (!gravity_aligned_frame_.empty()) {
    ground_points_msg->header.frame_id = gravity_aligned_frame_;
  }
  ground_points_pub_->publish(std::move(ground_points_msg));

  auto off_ground_points_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*off_ground_points, *off_ground_points_msg);
  off_ground_points_msg->header = msg->header;
  if (!gravity_aligned_frame_.empty()) {
    off_ground_points_msg->header.frame_id = gravity_aligned_frame_;
  }
  off_ground_points_pub_->publish(std::move(off_ground_points_msg));
}

void GroundFilter::initializeParameters()
{
  gravity_aligned_frame_ = declare_parameter<std::string>("gravity_aligned_frame", "");
  crop_range_min_ = declare_parameter<std::vector<double>>("crop_range.min", std::vector<double>{});
  crop_range_max_ = declare_parameter<std::vector<double>>("crop_range.max", std::vector<double>{});

  csf_.setClothResolution(declare_parameter<double>("cloth_resolution", csf_.getClothResolution()));
  csf_.setClothMargin(declare_parameter<double>("cloth_margin", csf_.getClothMargin()));
  csf_.setClothRigidness(declare_parameter<int>("cloth_rigidness", csf_.getClothRigidness()));
  csf_.setClothInitialZOffset(
    declare_parameter<double>("cloth_initial_z_offset", csf_.getClothInitialZOffset()));
  csf_.setGravity(declare_parameter<double>("gravity", csf_.getGravity()));
  csf_.setTimeStep(declare_parameter<double>("time_step", csf_.getTimeStep()));
  csf_.setMaxIterations(declare_parameter<int>("max_iterations", csf_.getMaxIterations()));
  csf_.setIterationTerminationThreshold(
    declare_parameter<double>(
      "iteration_termination_threshold", csf_.getIterationTerminationThreshold()));
  csf_.enablePostProcessing(
    declare_parameter<bool>("enable_post_processing", csf_.isPostProcessingEnabled()));
  csf_.setSlopeFittingThreshold(
    declare_parameter<double>("slope_fitting_threshold", csf_.getSlopeFittingThreshold()));
  csf_.setClassThreshold(
    declare_parameter<double>("classification_threshold", csf_.getClassThreshold()));

  parameter_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
    "crop_range.min",
    [this](const rclcpp::Parameter & param) { crop_range_min_ = param.as_double_array(); }));
  parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
    "crop_range.max",
    [this](const rclcpp::Parameter & param) { crop_range_max_ = param.as_double_array(); }));
  parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
    "cloth_resolution",
    [this](const rclcpp::Parameter & param) { csf_.setClothResolution(param.as_double()); }));
  parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
    "cloth_margin",
    [this](const rclcpp::Parameter & param) { csf_.setClothMargin(param.as_double()); }));
  parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
    "cloth_rigidness",
    [this](const rclcpp::Parameter & param) { csf_.setClothRigidness(param.as_int()); }));
  parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
    "cloth_initial_z_offset",
    [this](const rclcpp::Parameter & param) { csf_.setClothInitialZOffset(param.as_double()); }));
  parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
    "gravity", [this](const rclcpp::Parameter & param) { csf_.setGravity(param.as_double()); }));
  parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
    "time_step", [this](const rclcpp::Parameter & param) { csf_.setTimeStep(param.as_double()); }));
  parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
    "max_iterations",
    [this](const rclcpp::Parameter & param) { csf_.setMaxIterations(param.as_int()); }));
  parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
    "iteration_termination_threshold", [this](const rclcpp::Parameter & param) {
      csf_.setIterationTerminationThreshold(param.as_double());
    }));
  parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
    "enable_post_processing",
    [this](const rclcpp::Parameter & param) { csf_.enablePostProcessing(param.as_bool()); }));
  parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
    "slope_fitting_threshold",
    [this](const rclcpp::Parameter & param) { csf_.setSlopeFittingThreshold(param.as_double()); }));
  parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
    "classification_threshold",
    [this](const rclcpp::Parameter & param) { csf_.setClassThreshold(param.as_double()); }));
}
}  // namespace csf_ros

RCLCPP_COMPONENTS_REGISTER_NODE(csf_ros::GroundFilter)
