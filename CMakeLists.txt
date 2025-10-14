cmake_minimum_required(VERSION 3.8)
project(csf_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)
target_link_libraries(${PROJECT_NAME}
  csf_pcl::csf_pcl
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "csf_ros::GroundFilter"
  EXECUTABLE ground_filter_node
)

ament_auto_package()
