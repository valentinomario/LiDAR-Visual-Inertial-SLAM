cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0074 NEW)
project(lidar_odometry)
message(STATUS "cloud_messages: " ${cloud_msg_INCLUDE_DIRS})

# CMake flags
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-Wall -pthread")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -pthread")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
set(OpenCV_STATIC ON)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenMP REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# Include directories
include_directories(
        include/lidar_odometry
)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/CloudInfo.msg" "srv/SaveMap.srv" DEPENDENCIES std_msgs sensor_msgs)

# IMU Preintegration
add_executable(imuPreintegration src/imuPreintegration.cpp)
ament_target_dependencies(imuPreintegration
        rclcpp
        rclpy
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        pcl_conversions
        pcl_msgs
        visualization_msgs
        tf2
        tf2_ros
        tf2_eigen
        tf2_geometry_msgs
        OpenCV
        PCL
        GTSAM
        Eigen
        livox_ros_driver2
)
target_link_libraries(imuPreintegration gtsam "${cpp_typesupport_target}")

# Image projection
add_executable(imageProjection src/imageProjection.cpp)
ament_target_dependencies(imageProjection
        rclcpp
        rclpy
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        pcl_conversions
        pcl_msgs
        visualization_msgs
        tf2
        tf2_ros
        tf2_eigen
        tf2_sensor_msgs
        tf2_geometry_msgs
        OpenCV
        PCL
        livox_ros_driver2
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(imageProjection "${cpp_typesupport_target}")


add_executable(featureExtraction src/featureExtraction.cpp)
ament_target_dependencies(featureExtraction
        rclcpp
        rclpy
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        pcl_conversions
        pcl_msgs
        visualization_msgs
        tf2
        tf2_ros
        tf2_eigen
        tf2_sensor_msgs
        tf2_geometry_msgs
        OpenCV
        PCL
        livox_ros_driver2
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(featureExtraction "${cpp_typesupport_target}")

# Mapping Optimization
add_executable(mapOptimization src/mapOptimization.cpp)
ament_target_dependencies(mapOptimization
        rclcpp
        rclpy
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        pcl_conversions
        pcl_msgs
        visualization_msgs
        tf2
        tf2_ros
        tf2_eigen
        tf2_sensor_msgs
        tf2_geometry_msgs
        OpenCV
        PCL
        GTSAM
        livox_ros_driver2
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

if (OpenMP_CXX_FOUND)
  target_link_libraries(mapOptimization gtsam "${cpp_typesupport_target}" OpenMP::OpenMP_CXX)
else()
  target_link_libraries(mapOptimization gtsam "${cpp_typesupport_target}")
endif()

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
        imuPreintegration
        imageProjection
        featureExtraction
        mapOptimization

        DESTINATION lib/${PROJECT_NAME}
)

# install(
#        DIRECTORY "include/"
#        DESTINATION include
# )

ament_export_include_directories(include)

ament_export_dependencies(rosidl_default_runtime)
# Finalize
ament_package()


message(STATUS "===== Selected CMake Variables =====")

if(CMAKE_BUILD_TYPE)
  message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")
else()
  message(WARNING "CMAKE_BUILD_TYPE is not set")
endif()

if(CMAKE_CXX_FLAGS)
  message(STATUS "CMAKE_CXX_FLAGS = ${CMAKE_CXX_FLAGS}")
else()
  message(WARNING "CMAKE_CXX_FLAGS is not set")
endif()

if(CMAKE_CXX_FLAGS_RELEASE)
  message(STATUS "CMAKE_CXX_FLAGS_RELEASE = ${CMAKE_CXX_FLAGS_RELEASE}")
else()
  message(WARNING "CMAKE_CXX_FLAGS_RELEASE is not set")
endif()

# Pacchetti trovati
if(PCL_INCLUDE_DIRS)
  message(STATUS "PCL_INCLUDE_DIRS = ${PCL_INCLUDE_DIRS}")
else()
  message(WARNING "PCL_INCLUDE_DIRS is not set")
endif()

if(OpenCV_INCLUDE_DIRS)
  message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
else()
  message(WARNING "OpenCV_INCLUDE_DIRS is not set")
endif()

if(CERES_INCLUDE_DIRS)
  message(STATUS "CERES_INCLUDE_DIRS = ${CERES_INCLUDE_DIRS}")
else()
  message(WARNING "CERES_INCLUDE_DIRS is not set")
endif()

if(Boost_INCLUDE_DIRS)
  message(STATUS "Boost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}")
else()
  message(WARNING "Boost_INCLUDE_DIRS is not set")
endif()

if(GTSAM_INCLUDE_DIR)
  message(STATUS "GTSAM_INCLUDE_DIR = ${GTSAM_INCLUDE_DIR}")
else()
  message(WARNING "GTSAM_INCLUDE_DIR is not set")
endif()

if(PCL_LIBRARY_DIRS)
  message(STATUS "PCL_LIBRARY_DIRS = ${PCL_LIBRARY_DIRS}")
else()
  message(WARNING "PCL_LIBRARY_DIRS is not set")
endif()

# if(OpenCV_LIBRARY_DIRS)
#   message(STATUS "OpenCV_LIBRARY_DIRS = ${OpenCV_LIBRARY_DIRS}")
# else()
#   message(WARNING "OpenCV_LIBRARY_DIRS is not set")
# endif()

# if(GTSAM_LIBRARY_DIRS)
#   message(STATUS "GTSAM_LIBRARY_DIRS = ${GTSAM_LIBRARY_DIRS}")
# else()
#   message(WARNING "GTSAM_LIBRARY_DIR is not set")
# endif()

if(PCL_LIBRARIES)
  message(STATUS "PCL_LIBRARIES = ${PCL_LIBRARIES}")
else()
  message(WARNING "PCL_LIBRARIES is not set")
endif()

if(OpenCV_LIBS)
  message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")
else()
  message(WARNING "OpenCV_LIBS is not set")
endif()

# if(CERES_LIBRARIES)
#   message(STATUS "CERES_LIBRARIES = ${CERES_LIBRARIES}")
# else()
#   message(WARNING "CERES_LIBRARIES is not set")
# endif()

if(Boost_LIBRARIES)
  message(STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}")
else()
  message(WARNING "Boost_LIBRARIES is not set")
endif()

if(OpenMP_CXX_FLAGS)
  message(STATUS "OpenMP_CXX_FLAGS = ${OpenMP_CXX_FLAGS}")
else()
  message(WARNING "OpenMP_CXX_FLAGS is not set")
endif()

if(OpenCV_LIBS)
  message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")
else()
  message(WARNING "OpenCV_LIBS is not set")
endif()

message(STATUS "===== End of Selected CMake Variables =====")
