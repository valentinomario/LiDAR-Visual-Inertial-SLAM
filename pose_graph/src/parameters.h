#pragma once

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/detail/float64_multi_array__struct.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>

extern camodocal::CameraPtr m_camera;
extern Eigen::Vector3d tic;
extern Eigen::Matrix3d qic;

extern std::string IMAGE_TOPIC;
extern int DEBUG_IMAGE;
extern int LOOP_CLOSURE;
extern double MATCH_IMAGE_SCALE;

extern rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match_img;
extern rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_match_msg;
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_key_pose;


