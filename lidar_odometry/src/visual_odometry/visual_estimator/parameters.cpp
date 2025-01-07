#include "parameters.h"

std::string PROJECT_NAME;

double INIT_DEPTH;
double MIN_PARALLAX;
double IMU_FREQ;
double DELTA_T_IMU;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string IMU_TOPIC;
double ROW, COL;
double TD, TR;

int USE_LIDAR;
int ALIGN_CAMERA_LIDAR_COORDINATE;

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

std::vector<double> extRotV_lidar2imu;
std::vector<double> extTransV_lidar2imu;
Eigen::Matrix3d extRot_lidar2imu;
Eigen::Vector3d extTrans_lidar2imu;


void readParameters(const std::string& config_path)
{

    std::string config_file = config_path + "/params_camera.yaml";
    std::string config_file1 = config_path + "/params_camera1.yaml";
    std::string config_file2 = config_path + "/params_camera2.yaml";

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    cv::FileStorage fsSettings1(config_file1, cv::FileStorage::READ);
    cv::FileStorage fsSettings2(config_file2, cv::FileStorage::READ);

    /*if (!fsSettings.isOpened() || !fsSettings1.isOpened() || !fsSettings2.isOpened())
    {
        RCLCPP_ERROR(node->get_logger(), "ERROR: Wrong path to settings.");
        return;
    }*/

    fsSettings["project_name"] >> PROJECT_NAME;

    // Extract package share directory
    //std::string pkg_path = ament_index_cpp::get_package_share_directory(PROJECT_NAME);
    std::string pkg_path = "declare parameter";
    fsSettings["imu_topic"] >> IMU_TOPIC;

    fsSettings["use_lidar"] >> USE_LIDAR;
    fsSettings["align_camera_lidar_estimation"] >> ALIGN_CAMERA_LIDAR_COORDINATE;

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

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

    IMU_FREQ = fsSettings["imu_freq"];
    DELTA_T_IMU = 1.0 / IMU_FREQ;
    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    // TODO: ESTIMATE_EXTRINSIC is always 0 in our case
    cv::Mat cv_R, cv_T;
    fsSettings["extrinsicRotation"] >> cv_R;
    fsSettings["extrinsicTranslation"] >> cv_T;
    Eigen::Matrix3d eigen_R;
    Eigen::Vector3d eigen_T;
    cv::cv2eigen(cv_R, eigen_R);
    cv::cv2eigen(cv_T, eigen_T);
    RIC.push_back(eigen_R);
    TIC.push_back(eigen_T);

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    /*if (ESTIMATE_TD)
        RCLCPP_INFO(node->get_logger(), "Unsynchronized sensors. Initial TD: %f", TD);
    else
        RCLCPP_INFO(node->get_logger(), "Synchronized sensors. TD: %f", TD);
    */
    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        // RCLCPP_INFO(node->get_logger(), "Rolling shutter enabled. Readout time per line: %f", TR);
    }
    else
    {
        TR = 0.0;
    }

    fsSettings.release();
    fsSettings1.release();
    fsSettings2.release();
}