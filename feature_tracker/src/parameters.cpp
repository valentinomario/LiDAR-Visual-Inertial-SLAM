#include "parameters.h"

std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::string POINT_CLOUD_TOPIC;
std::string PROJECT_NAME;

bool USE_GPU;
bool USE_GPU_ACCELERATED_FLOW;

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

double L_C_TX;
double L_C_TY;
double L_C_TZ;
double L_C_RX;
double L_C_RY;
double L_C_RZ;

int USE_LIDAR;
int LIDAR_SKIP;

template <typename T>
T readParam(rclcpp::Node::SharedPtr n, std::string name)
{
    T ans;

    std::string default_value = "";
    n->declare_parameter<std::string>(name, default_value);
    if (n->get_parameter(name, ans))
    {
        RCLCPP_INFO_STREAM(n->get_logger(), "Loaded " << name << ": " << ans);
    }
    else
    {
        RCLCPP_ERROR_STREAM(n->get_logger(), "Failed to load " << name);
        rclcpp::shutdown();
    }
    return ans;
}

void readParameters(rclcpp::Node::SharedPtr &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    RCUTILS_LOG_INFO("config_file: %s", config_file.c_str());
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        RCUTILS_LOG_ERROR("ERROR: Wrong path to settings");
    }
    std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");
    fsSettings["project_name"] >> PROJECT_NAME;
    fsSettings["point_cloud_topic"] >> POINT_CLOUD_TOPIC;

    fsSettings["use_gpu"] >> USE_GPU;
    fsSettings["use_gpu_accelerated_flow"] >> USE_GPU_ACCELERATED_FLOW;

    // lidar configurations
    fsSettings["use_lidar"] >> USE_LIDAR;
    fsSettings["lidar_skip"] >> LIDAR_SKIP;

    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["imu_topic"] >> IMU_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];

    L_C_TX = fsSettings["lidar_to_cam_tx"];
    L_C_TY = fsSettings["lidar_to_cam_ty"];
    L_C_TZ = fsSettings["lidar_to_cam_tz"];
    L_C_RX = fsSettings["lidar_to_cam_rx"];
    L_C_RY = fsSettings["lidar_to_cam_ry"];
    L_C_RZ = fsSettings["lidar_to_cam_rz"];

    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
    {
        std::string mask_name;
        fsSettings["fisheye_mask"] >> mask_name;
        FISHEYE_MASK = VINS_FOLDER_PATH + mask_name;
        RCLCPP_INFO(n->get_logger(), "FISHEYE mask: %s", mask_name.c_str());
    }
    CAM_NAMES.push_back(config_file);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();


}

float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, rclcpp::Time thisStamp, std::string thisFrame)
{
    if (thisPub->get_subscription_count() == 0)
        return;
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    thisPub->publish(tempCloud);
}
