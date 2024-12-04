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
    string gpsTopic;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Lidar Sensor Configuration: Velodyne
    int N_SCAN;
    int Horizon_SCAN;
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
};

template<typename T>
sensor_msgs::msg::PointCloud2 publishCloud(std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> publisher,
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