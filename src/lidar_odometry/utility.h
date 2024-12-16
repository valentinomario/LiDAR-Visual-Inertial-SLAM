#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/buffer.h>

#include <Eigen/Dense>

#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include "imu_tracker.h"

using namespace std;

typedef pcl::PointXYZI PointType;


class ParamServer
{
public:

    std::string PROJECT_NAME;

    std::string robot_id;

    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    // string gpsTopic;

    // GPS Settings
    bool useImuHeadingInitialization;
    // bool useGpsElevation;
    // float gpsCovThreshold;
    // float poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Lidar Sensor Configuration: Velodyne
    // int N_SCAN;
    // int Horizon_SCAN;
    float ang_res_y;
    int lidar_type;
    string timeField;
    int downsampleRate;
    float lidarMaxRange;
    float lidarMinRange;
    int feature_enable;
    int remove_noise;

    // noise removal
    int min_cluster_size;
    int segment_valid_point_num;
    int segment_valid_line_num;

    float fov_min_theta;
    float fov_max_theta;
    int HRES;
    int VRES;

    // IMU
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;

    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    Eigen::Quaterniond q_lidar2imu;
    Eigen::Matrix3d R_imu2lidar;
    Eigen::Quaterniond q_imu2lidar;
    Eigen::Vector3d t_imu2lidar;


    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    float z_tolerance;
    float rotation_tolerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold;
    float surroundingkeyframeAddingAngleThreshold;
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;

    // Loop closure
    bool loopClosureEnableFlag;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ParamServer()
    {

    }



    sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& imu_in,
                                std::shared_ptr<ImuTracker> imu_tracker)
    {
    sensor_msgs::msg::Imu imu_out = imu_in;
    {
        // rotate acceleration
        Eigen::Vector3d imu_linear_acceleration(imu_in.linear_acceleration.x,
                            imu_in.linear_acceleration.y,
                            imu_in.linear_acceleration.z);
        imu_linear_acceleration = R_imu2lidar * imu_linear_acceleration;
        imu_out.linear_acceleration.x = imu_linear_acceleration.x();
        imu_out.linear_acceleration.y = imu_linear_acceleration.y();
        imu_out.linear_acceleration.z = imu_linear_acceleration.z();

        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = R_imu2lidar * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();

        // rotate roll pitch yaw
        Eigen::Vector3d imu_angular_velocity(imu_in.angular_velocity.x,
                            imu_in.angular_velocity.y,
                            imu_in.angular_velocity.z);

        imu_tracker->Advance(imu_in.header.stamp.sec + imu_in.header.stamp.nanosec * 1e-9);
        imu_tracker->AddImuLinearAccelerationObservation(imu_linear_acceleration);
        imu_tracker->AddImuAngularVelocityObservation(imu_angular_velocity);
        Eigen::Quaterniond q_from = imu_tracker->orientation();
        Eigen::Quaterniond q_final = q_from * q_lidar2imu;

        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        // cout << std::fixed << "imu  angular velocity: " << imu_in.header.stamp.toSec() << " "
        //                                                 << gyr.x() << " "
        //                                                 << gyr.y() << " "
        //                                                 << gyr.z() << endl;

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            // RCLCPP_ERROR(node->get_logger(), "Invalid quaternion, please use a 9-axis IMU!");
            rclcpp::shutdown();
        }
    }
    return imu_out;
    }

    void declareParameters(std::shared_ptr<rclcpp::Node> node)
    {
        // TODO set defaults
        node->declare_parameter<std::string>("PROJECT_NAME", "emv_lio2");
        node->declare_parameter<std::string>("robot_id", "roboat");
        node->declare_parameter<std::string>("pointCloudTopic", "/livox/lidar");
        node->declare_parameter<std::string>("imuTopic", "/livox/imu");
        node->declare_parameter<std::string>("odomTopic", "odometry/imu");
        // node->declare_parameter<std::string>("gpsTopic", "odometry/gps");

        node->declare_parameter<bool>("useImuHeadingInitialization", false);
        // node->declare_parameter<bool>("useGpsElevation", false);
        // node->declare_parameter<float>("gpsCovThreshold", 2.0);
        // node->declare_parameter<float>("poseCovThreshold", 25.0);

        node->declare_parameter<bool>("savePCD", false);
        node->declare_parameter<std::string>("savePCDDirectory", "/tmp/loam/");

        // node->declare_parameter<int>("N_SCAN", 32);
        // node->declare_parameter<int>("Horizon_SCAN", 2000);
        node->declare_parameter<float>("ang_res_y", 1.0);
        node->declare_parameter<int>("lidar_type", 2);
        node->declare_parameter<std::string>("timeField", "timestamp");
        node->declare_parameter<int>("downsampleRate", 2);
        node->declare_parameter<float>("lidarMaxRange", 50.0);
        node->declare_parameter<float>("lidarMinRange", 0.5);
        node->declare_parameter<int>("feature_enable", 0);
        node->declare_parameter<int>("remove_noise", 0);

        node->declare_parameter<float>("fov_min_theta", 36.0);
        node->declare_parameter<float>("fov_max_theta", 98.0);
        node->declare_parameter<int>("VRES", 40);
        node->declare_parameter<int>("HRES", 2000);

        node->declare_parameter<int>("min_cluster_size", 10);
        node->declare_parameter<int>("segment_valid_point_num", 5);
        node->declare_parameter<int>("segment_valid_line_num", 3);

        node->declare_parameter<float>("imuAccNoise", 0.01);
        node->declare_parameter<float>("imuGyrNoise", 0.001);
        node->declare_parameter<float>("imuAccBiasN", 0.0002);
        node->declare_parameter<float>("imuGyrBiasN", 0.00003);
        node->declare_parameter<float>("imuGravity", 9.80511);
        node->declare_parameter<std::vector<double>>("extrinsicRot", {1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0});
        node->declare_parameter<std::vector<double>>("extrinsicRPY", {0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0});
        node->declare_parameter<std::vector<double>>("extrinsicTrans", {0.0,0.0,0.0});

        node->declare_parameter<float>("edgeThreshold", 1.0);
        node->declare_parameter<float>("surfThreshold", 0.1);
        node->declare_parameter<int>("edgeFeatureMinValidNum", -1);
        node->declare_parameter<int>("surfFeatureMinValidNum", 100);

        node->declare_parameter<float>("odometrySurfLeafSize", 0.4);
        node->declare_parameter<float>("mappingCornerLeafSize", 0.2);
        node->declare_parameter<float>("mappingSurfLeafSize", 0.4);

        node->declare_parameter<float>("z_tolerance", 1000);
        node->declare_parameter<float>("rotation_tolerance", 1000);

        node->declare_parameter<int>("numberOfCores", 4);
        node->declare_parameter<double>("mappingProcessInterval", 0.05);

        node->declare_parameter<float>("surroundingkeyframeAddingDistThreshold", 1.0);
        node->declare_parameter<float>("surroundingkeyframeAddingAngleThreshold", 0.2);
        node->declare_parameter<float>("surroundingKeyframeDensity", 2.0);
        node->declare_parameter<float>("surroundingKeyframeSearchRadius", 50.0);

        node->declare_parameter<bool>("loopClosureEnableFlag", false);
        node->declare_parameter<int>("surroundingKeyframeSize", 25);
        node->declare_parameter<float>("historyKeyframeSearchRadius", 20.0);
        node->declare_parameter<float>("historyKeyframeSearchTimeDiff", 30.0);
        node->declare_parameter<int>("historyKeyframeSearchNum", 25);
        node->declare_parameter<float>("historyKeyframeFitnessScore", 0.3);

        node->declare_parameter<float>("globalMapVisualizationSearchRadius", 1e3);
        node->declare_parameter<float>("globalMapVisualizationPoseDensity", 10.0);
        node->declare_parameter<float>("globalMapVisualizationLeafSize", 1.0);

        node->declare_parameter<int>("paramsCheck", 0);
    }

    void getParameters(std::shared_ptr<rclcpp::Node> node) {
        node->get_parameter("PROJECT_NAME", PROJECT_NAME);
        node->get_parameter("robot_id", robot_id);
        node->get_parameter("pointCloudTopic", pointCloudTopic);
        node->get_parameter("imuTopic", imuTopic);
        node->get_parameter("odomTopic", odomTopic);
        // node->get_parameter("gpsTopic", gpsTopic);

        node->get_parameter("useImuHeadingInitialization", useImuHeadingInitialization);
        // node->get_parameter("useGpsElevation", useGpsElevation);
        // node->get_parameter("gpsCovThreshold", gpsCovThreshold);
        // node->get_parameter("poseCovThreshold", poseCovThreshold);

        node->get_parameter("savePCD", savePCD);
        node->get_parameter("savePCDDirectory", savePCDDirectory);

        //node->get_parameter("N_SCAN", N_SCAN);
        //node->get_parameter("Horizon_SCAN", Horizon_SCAN);
        node->get_parameter("ang_res_y", ang_res_y);
        node->get_parameter("lidar_type", lidar_type);
        node->get_parameter("timeField", timeField);
        node->get_parameter("downsampleRate", downsampleRate);
        node->get_parameter("lidarMaxRange", lidarMaxRange);
        node->get_parameter("lidarMinRange", lidarMinRange);
        node->get_parameter("feature_enable", feature_enable);
        node->get_parameter("remove_noise", remove_noise);

        node->get_parameter("fov_min_theta", fov_min_theta);
        node->get_parameter("fov_max_theta", fov_max_theta);
        node->get_parameter("VRES", VRES);
        node->get_parameter("HRES", HRES);

        node->get_parameter("min_cluster_size", min_cluster_size);
        node->get_parameter("segment_valid_point_num", segment_valid_point_num);
        node->get_parameter("segment_valid_line_num", segment_valid_line_num);

        node->get_parameter("imuAccNoise", imuAccNoise);
        node->get_parameter("imuGyrNoise", imuGyrNoise);
        node->get_parameter("imuAccBiasN", imuAccBiasN);
        node->get_parameter("imuGyrBiasN", imuGyrBiasN);
        node->get_parameter("imuGravity", imuGravity);
        node->get_parameter("extrinsicRot", extRotV);
        node->get_parameter("extrinsicRPY", extRPYV);
        node->get_parameter("extrinsicTrans", extTransV);

        node->get_parameter("edgeThreshold", edgeThreshold);
        node->get_parameter("surfThreshold", surfThreshold);
        node->get_parameter("edgeFeatureMinValidNum", edgeFeatureMinValidNum);
        node->get_parameter("surfFeatureMinValidNum", surfFeatureMinValidNum);

        node->get_parameter("odometrySurfLeafSize", odometrySurfLeafSize);
        node->get_parameter("mappingCornerLeafSize", mappingCornerLeafSize);
        node->get_parameter("mappingSurfLeafSize", mappingSurfLeafSize);

        node->get_parameter("z_tolerance", z_tolerance);
        node->get_parameter("rotation_tolerance", rotation_tolerance);

        node->get_parameter("numberOfCores", numberOfCores);
        node->get_parameter("mappingProcessInterval", mappingProcessInterval);

        node->get_parameter("surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold);
        node->get_parameter("surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold);
        node->get_parameter("surroundingKeyframeDensity", surroundingKeyframeDensity);
        node->get_parameter("surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius);

        node->get_parameter("loopClosureEnableFlag", loopClosureEnableFlag);
        node->get_parameter("surroundingKeyframeSize", surroundingKeyframeSize);
        node->get_parameter("historyKeyframeSearchRadius", historyKeyframeSearchRadius);
        node->get_parameter("historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff);
        node->get_parameter("historyKeyframeSearchNum", historyKeyframeSearchNum);
        node->get_parameter("historyKeyframeFitnessScore", historyKeyframeFitnessScore);

        node->get_parameter("globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius);
        node->get_parameter("globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity);
        node->get_parameter("globalMapVisualizationLeafSize", globalMapVisualizationLeafSize);

        int paramsCheck;
        node->get_parameter("paramsCheck", paramsCheck);

        if(paramsCheck != 69) RCLCPP_ERROR(node->get_logger(), "Error loading parameters from config file");

        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);

        q_lidar2imu = extRot;
        R_imu2lidar = extRot.inverse();
        q_imu2lidar = R_imu2lidar;
        t_imu2lidar = -R_imu2lidar * extTrans;

    }
};

template<typename T>
sensor_msgs::msg::PointCloud2 publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
    T cloud,
    rclcpp::Time timeStamp,
    std::string frame)
{
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*cloud, tempCloud);
    tempCloud.header.stamp = timeStamp;
    tempCloud.header.frame_id = frame;
    if (publisher->get_subscription_count() != 0)
        publisher->publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::msg::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::msg::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::msg::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf2::Quaternion orientation;
    tf2::fromMsg(thisImuMsg->orientation, orientation); // TODO check if it's correct
    tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
    //RCLCPP_INFO(node->get_logger(), "Check if values match: translation %f, %f, %f", t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
    //RCLCPP_INFO(node->get_logger(), ".. and ........................... %f, %f, %f", transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
//
    //RCLCPP_INFO(node->get_logger(), "Check if values match: rotation %f, %f, %f, %f", t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
    //RCLCPP_INFO(node->get_logger(), ".. and ........................ %f, %f, %f, %f", transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ(), transform.getRotation().getW());

    // cout << "imuYaw: " << imuYaw << endl;

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}


float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

#endif