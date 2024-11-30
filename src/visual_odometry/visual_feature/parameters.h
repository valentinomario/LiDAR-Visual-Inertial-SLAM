#pragma once

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/random_sample.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <mutex>
#include <cmath>
#include <glog/logging.h>

using namespace std;

typedef pcl::PointXYZI PointType;

// Dichiarazioni dei parametri globali
extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;

// Parametri definiti nel file YAML
extern int NUM_OF_CAM;
extern int num_of_cam;
extern std::string PROJECT_NAME;
extern std::string IMAGE_TOPIC_0;
extern std::string IMAGE_TOPIC_1;
extern std::string IMAGE_TOPIC_2;
extern std::string IMU_TOPIC;
extern std::string POINT_CLOUD_TOPIC;

extern int USE_LIDAR;
extern int LIDAR_SKIP;

extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;

extern double C_L_TX;
extern double C_L_TY;
extern double C_L_TZ;
extern double C_L_RX;
extern double C_L_RY;
extern double C_L_RZ;

extern double C_L_TX1;
extern double C_L_TY1;
extern double C_L_TZ1;
extern double C_L_RX1;
extern double C_L_RY1;
extern double C_L_RZ1;

extern double C_L_TX2;
extern double C_L_TY2;
extern double C_L_TZ2;
extern double C_L_RX2;
extern double C_L_RY2;
extern double C_L_RZ2;

extern Eigen::Matrix3d extRot_lidar2imu;
extern Eigen::Vector3d extTrans_lidar2imu;

// Funzione per leggere i parametri
void readParameters(const std::string& config_path);

// Calcola la distanza tra punti
float pointDistance(PointType p);

float pointDistance(PointType p1, PointType p2);

// Pubblica una nuvola di punti
void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
                  pcl::PointCloud<PointType>::Ptr cloud,
                  rclcpp::Time stamp,
                  std::string frame_id);
