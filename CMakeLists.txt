cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0074 NEW)
project(emv_lio2)

# CMake flags
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -pthread -w")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread -w")


# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(OpenMP REQUIRED)

# Include directories
include_directories(
        include
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
)

link_libraries(${OpenCV_LIBS}
        ceres
        gtsam
)

link_directories(
        ${PCL_LIBRARY_DIRS}
        #${OpenCV_LIBRARY_DIRS}
        # ${GTSAM_LIBRARY_DIRS}
)

# Test
add_executable(test_node src/test_node.cpp)
ament_target_dependencies(test_node rclcpp std_msgs)
install(TARGETS test_node DESTINATION lib/${PROJECT_NAME})


# Visual Feature Tracker
add_executable(${PROJECT_NAME}_visual_feature
        src/visual_odometry/visual_feature/feature_tracker.cpp
        src/visual_odometry/visual_feature/feature_tracker_node.cpp
        src/visual_odometry/visual_feature/parameters.cpp
        src/visual_odometry/visual_feature/camera_models/Camera.cc
        src/visual_odometry/visual_feature/camera_models/CostFunctionFactory.cc
        src/visual_odometry/visual_feature/camera_models/ScaramuzzaCamera.cc
        src/visual_odometry/visual_feature/camera_models/CameraFactory.cc
        src/visual_odometry/visual_feature/camera_models/EquidistantCamera.cc
        src/visual_odometry/visual_feature/camera_models/gpl.cc
        src/visual_odometry/visual_feature/camera_models/CataCamera.cc
        src/visual_odometry/visual_feature/camera_models/PinholeCamera.cc
)

ament_target_dependencies(${PROJECT_NAME}_visual_feature
        rclcpp
        sensor_msgs
        std_msgs
        geometry_msgs
        nav_msgs
        tf2
        tf2_ros
        tf2_eigen
        PCL
        pcl_conversions
        cv_bridge
        OpenCV
        Ceres
)


# Visual Odometry
# add_executable(${PROJECT_NAME}_visual_odometry src/visual_odometry/visual_estimator/*.cpp src/visual_odometry/visual_estimator/factor/*.cpp src/visual_odometry/visual_estimator/initial/*.cpp src/visual_odometry/visual_estimator/utility/*.cpp)
# ament_target_dependencies(${PROJECT_NAME}_visual_odometry rclcpp sensor_msgs std_msgs PCL OpenCV Ceres)

# Visual Loop
# add_executable(${PROJECT_NAME}_visual_loop src/visual_odometry/visual_loop/*.cpp src/visual_odometry/visual_loop/utility/*.cpp src/visual_odometry/visual_loop/ThirdParty/*.cpp src/visual_odometry/visual_loop/ThirdParty/DBoW/*.cpp src/visual_odometry/visual_loop/ThirdParty/DUtils/*.cpp src/visual_odometry/visual_loop/ThirdParty/DVision/*.cpp src/visual_odometry/visual_feature/camera_models/*.cc)
# ament_target_dependencies(${PROJECT_NAME}_visual_loop rclcpp sensor_msgs std_msgs PCL OpenCV Ceres)

# IMU Preintegration
# add_executable(${PROJECT_NAME}_imuPreintegration src/lidar_odometry/imuPreintegration.cpp src/lidar_odometry/imu_tracker.cc)
# ament_target_dependencies(${PROJECT_NAME}_imuPreintegration rclcpp sensor_msgs std_msgs PCL GTSAM)

# Range Image Projection
# add_executable(${PROJECT_NAME}_imageProjection src/lidar_odometry/imageProjection.cpp src/lidar_odometry/imu_tracker.cc)
# ament_target_dependencies(${PROJECT_NAME}_imageProjection rclcpp sensor_msgs std_msgs PCL)

# Feature Association
# add_executable(${PROJECT_NAME}_featureExtraction src/lidar_odometry/featureExtraction.cpp)
# ament_target_dependencies(${PROJECT_NAME}_featureExtraction rclcpp sensor_msgs std_msgs PCL)

# Mapping Optimization
# add_executable(${PROJECT_NAME}_mapOptimization src/lidar_odometry/mapOptmization.cpp include/ikd-Tree/ikd_Tree.cpp)
# ament_target_dependencies(${PROJECT_NAME}_mapOptimization rclcpp sensor_msgs std_msgs PCL GTSAM OpenMP)
# target_compile_options(${PROJECT_NAME}_mapOptimization PRIVATE ${OpenMP_CXX_FLAGS})

# Test
# add_executable(${PROJECT_NAME}_msgTest src/lidar_odometry/msgTest.cpp)
# ament_target_dependencies(${PROJECT_NAME}_msgTest rclcpp sensor_msgs std_msgs PCL OpenCV)

# Install targets
# install(TARGETS
#         ${PROJECT_NAME}_visual_feature
#         ${PROJECT_NAME}_visual_odometry
#         ${PROJECT_NAME}_visual_loop
#         ${PROJECT_NAME}_imuPreintegration
#         ${PROJECT_NAME}_imageProjection
#         ${PROJECT_NAME}_featureExtraction
#         ${PROJECT_NAME}_mapOptimization
#         ${PROJECT_NAME}_msgTest
#         DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
        ${PROJECT_NAME}_visual_feature
        DESTINATION lib/${PROJECT_NAME}
)
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

# Librerie di pacchetti
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

if(CERES_LIBRARIES)
  message(STATUS "CERES_LIBRARIES = ${CERES_LIBRARIES}")
else()
  message(WARNING "CERES_LIBRARIES is not set")
endif()

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

# Finalize
ament_package()


