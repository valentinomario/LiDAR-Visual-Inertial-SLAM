#include "parameters.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

std::string IMAGE_TOPIC_0;
std::string IMAGE_TOPIC_1;
std::string IMAGE_TOPIC_2;
std::string IMU_TOPIC;
std::string POINT_CLOUD_TOPIC;
std::string PROJECT_NAME;

std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;

int NUM_OF_CAM;

double C_L_TX;
double C_L_TY;
double C_L_TZ;
double C_L_RX;
double C_L_RY;
double C_L_RZ;

double C_L_TX1;
double C_L_TY1;
double C_L_TZ1;
double C_L_RX1;
double C_L_RY1;
double C_L_RZ1;

double C_L_TX2;
double C_L_TY2;
double C_L_TZ2;
double C_L_RX2;
double C_L_RY2;
double C_L_RZ2;

int USE_LIDAR;
int LIDAR_SKIP;

std::vector<double> extRotV_lidar2imu;
std::vector<double> extTransV_lidar2imu;
Eigen::Matrix3d extRot_lidar2imu;
Eigen::Vector3d extTrans_lidar2imu;

void readParameters(std::shared_ptr<rclcpp::Node> node)
{
    // Lidars parameters
    node->declare_parameter<std::string>("PROJECT_NAME", "emv-lio2");
    node->get_parameter("PROJECT_NAME", PROJECT_NAME);

    extRotV_lidar2imu = node->declare_parameter<std::vector<double>>(PROJECT_NAME + ".extrinsicRot", {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0});
    extTransV_lidar2imu = node->declare_parameter<std::vector<double>>(PROJECT_NAME + ".extrinsicTrans", {0.0,0.0,0.0});
    extRot_lidar2imu = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV_lidar2imu.data(), 3, 3);
    extTrans_lidar2imu = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV_lidar2imu.data(), 3, 1);

    NUM_OF_CAM = node->declare_parameter<int>(PROJECT_NAME + ".NUM_OF_CAM", 1);
    RCLCPP_INFO(node->get_logger(), "\033[1;32mTEST\033[0m");

    // Load config files
    // std::string pkg_path = ament_index_cpp::get_package_share_directory(PROJECT_NAME);
    std::string config_path = node->declare_parameter<std::string>("config_dir", "/home/user/ros2_ws/install/emv-lio2/share/emv-lio2/config");
    std::string config_file = config_path + "/params_camera.yaml";
    std::string config_file1 = config_path + "/params_camera1.yaml";
    std::string config_file2 = config_path + "/params_camera2.yaml";

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    cv::FileStorage fsSettings1(config_file1, cv::FileStorage::READ);
    cv::FileStorage fsSettings2(config_file2, cv::FileStorage::READ);

    if (!fsSettings.isOpened() || !fsSettings1.isOpened() || !fsSettings2.isOpened())
    {
        RCLCPP_ERROR(node->get_logger(), "ERROR: Wrong path to settings: %s", config_path.c_str());
        return;
    }

    // Sensor topics
    fsSettings["image_topic"] >> IMAGE_TOPIC_0;
    fsSettings1["image_topic"] >> IMAGE_TOPIC_1;
    fsSettings2["image_topic"] >> IMAGE_TOPIC_2;
    fsSettings["imu_topic"] >> IMU_TOPIC;
    fsSettings["point_cloud_topic"] >> POINT_CLOUD_TOPIC;

    // Lidar configurations
    fsSettings["use_lidar"] >> USE_LIDAR;
    fsSettings["lidar_skip"] >> LIDAR_SKIP;

    // Feature and image settings
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];

    C_L_TX = fsSettings["cam_to_lidar_tx"];
    C_L_TY = fsSettings["cam_to_lidar_ty"];
    C_L_TZ = fsSettings["cam_to_lidar_tz"];
    C_L_RX = fsSettings["cam_to_lidar_rx"];
    C_L_RY = fsSettings["cam_to_lidar_ry"];
    C_L_RZ = fsSettings["cam_to_lidar_rz"];

    C_L_TX1 = fsSettings1["cam_to_lidar_tx"];
    C_L_TY1 = fsSettings1["cam_to_lidar_ty"];
    C_L_TZ1 = fsSettings1["cam_to_lidar_tz"];
    C_L_RX1 = fsSettings1["cam_to_lidar_rx"];
    C_L_RY1 = fsSettings1["cam_to_lidar_ry"];
    C_L_RZ1 = fsSettings1["cam_to_lidar_rz"];

    C_L_TX2 = fsSettings2["cam_to_lidar_tx"];
    C_L_TY2 = fsSettings2["cam_to_lidar_ty"];
    C_L_TZ2 = fsSettings2["cam_to_lidar_tz"];
    C_L_RX2 = fsSettings2["cam_to_lidar_rx"];
    C_L_RY2 = fsSettings2["cam_to_lidar_ry"];
    C_L_RZ2 = fsSettings2["cam_to_lidar_rz"];

    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
    {
        std::string mask_name;
        fsSettings["fisheye_mask"] >> mask_name;
        FISHEYE_MASK = config_path + mask_name;
    }

    CAM_NAMES = {config_file, config_file1, config_file2};

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();
    fsSettings1.release();
    fsSettings2.release();
}

float pointDistance(PointType p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
                  pcl::PointCloud<PointType>::Ptr cloud,
                  rclcpp::Time stamp,
                  std::string frame_id)
{
    if (publisher->get_subscription_count() == 0)
        return;

    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*cloud, tempCloud);
    tempCloud.header.stamp = stamp;
    tempCloud.header.frame_id = frame_id;
    publisher->publish(tempCloud);
}
