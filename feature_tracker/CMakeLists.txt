cmake_minimum_required(VERSION 3.5)
project(feature_tracker)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wextra -Wpedantic)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -pthread")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_model REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

include_directories("../camera_model/include")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(feature_tracker
    src/feature_tracker_node.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
    )

ament_target_dependencies(feature_tracker rclcpp rcpputils std_msgs sensor_msgs geometry_msgs cv_bridge camera_model image_transport PCL pcl_conversions tf2 tf2_ros tf2_geometry_msgs nav_msgs OpenCV livox_ros_driver2)
target_link_libraries(feature_tracker ${OpenCV_LIBS})

# Install nodes
install(
  TARGETS feature_tracker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()