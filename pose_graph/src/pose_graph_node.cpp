#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mutex>
#include <queue>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "loop_detector.h"
#include "utility/CameraPoseVisualization.h"
#include "parameters.h"
#define SKIP_FIRST_CNT 10
using namespace std;

queue<sensor_msgs::msg::Image::ConstPtr> image_buf;
queue<sensor_msgs::msg::PointCloud::ConstPtr> point_buf;
queue<nav_msgs::msg::Odometry::ConstPtr> pose_buf;

std::mutex m_buf;
std::mutex m_process;

LoopDetector loopDetector;

double SKIP_TIME = 0;
double SKIP_DIST = 0;

camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;

std::string IMAGE_TOPIC;

int DEBUG_IMAGE;
int LOOP_CLOSURE;
double MATCH_IMAGE_SCALE;

rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match_img;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_match_msg;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_key_pose;

BriefExtractor briefExtractor;

void new_sequence()
{
    m_buf.lock();
    while(!image_buf.empty())
        image_buf.pop();
    while(!point_buf.empty())
        point_buf.pop();
    while(!pose_buf.empty())
        pose_buf.pop();
    m_buf.unlock();
}

void image_callback(const sensor_msgs::msg::Image::ConstPtr image_msg)
{
    //ROS_INFO("image_callback!");
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    image_buf.push(image_msg);
    m_buf.unlock();
    //printf(" image time %f \n", image_msg->header.stamp.toSec());

    // detect unstable camera stream
    static double last_image_time = -1;
    if (last_image_time == -1)
        last_image_time = image_msg->header.stamp.sec+image_msg->header.stamp.nanosec * (1e-9);
    else if ((image_msg->header.stamp.sec+image_msg->header.stamp.nanosec * (1e-9)) - last_image_time > 1.0 || (image_msg->header.stamp.sec+image_msg->header.stamp.nanosec * (1e-9)) < last_image_time)
    {
        RCUTILS_LOG_WARN("image discontinue! detect a new sequence!");
        new_sequence();
    }
    last_image_time = image_msg->header.stamp.sec+image_msg->header.stamp.nanosec * (1e-9);
}

void point_callback(const sensor_msgs::msg::PointCloud::ConstPtr point_msg)
{
    //ROS_INFO("point_callback!");
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    point_buf.push(point_msg);
    m_buf.unlock();
}

void pose_callback(const nav_msgs::msg::Odometry::ConstPtr pose_msg)
{
    //ROS_INFO("pose_callback!");
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();

}

void extrinsic_callback(const nav_msgs::msg::Odometry::ConstPtr pose_msg)
{
    m_process.lock();
    tic = Vector3d(pose_msg->pose.pose.position.x,
                   pose_msg->pose.pose.position.y,
                   pose_msg->pose.pose.position.z);
    qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                      pose_msg->pose.pose.orientation.x,
                      pose_msg->pose.pose.orientation.y,
                      pose_msg->pose.pose.orientation.z).toRotationMatrix();
    m_process.unlock();
}

void process()
{
    if (!LOOP_CLOSURE)
        return;
    while (rclcpp::ok())
    {
        sensor_msgs::msg::Image::ConstPtr image_msg = NULL;
        sensor_msgs::msg::PointCloud::ConstPtr point_msg = NULL;
        nav_msgs::msg::Odometry::ConstPtr pose_msg = NULL;

        // find out the messages with same time stamp
        m_buf.lock();
        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if ((image_buf.front()->header.stamp.sec+image_buf.front()->header.stamp.nanosec * (1e-9)) > (pose_buf.front()->header.stamp.sec+pose_buf.front()->header.stamp.nanosec * (1e-9)))
            {
                pose_buf.pop();
                printf("throw pose at beginning\n");
            }
            else if ((image_buf.front()->header.stamp.sec+image_buf.front()->header.stamp.nanosec * (1e-9)) > (point_buf.front()->header.stamp.sec+point_buf.front()->header.stamp.nanosec * (1e-9)))
            {
                point_buf.pop();
                printf("throw point at beginning\n");
            }
            else if ((image_buf.back()->header.stamp.sec+image_buf.back()->header.stamp.nanosec * (1e-9)) >= (pose_buf.front()->header.stamp.sec+pose_buf.front()->header.stamp.nanosec * (1e-9)) 
                && (point_buf.back()->header.stamp.sec+point_buf.back()->header.stamp.nanosec * (1e-9)) >= (pose_buf.front()->header.stamp.sec+pose_buf.front()->header.stamp.nanosec * (1e-9)))
            {
                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty())
                    pose_buf.pop();
                while ((image_buf.front()->header.stamp.sec+image_buf.front()->header.stamp.nanosec * (1e-9)) < (pose_msg->header.stamp.sec+pose_msg->header.stamp.nanosec * (1e-9)))
                    image_buf.pop();
                image_msg = image_buf.front();
                image_buf.pop();

                while ((point_buf.front()->header.stamp.sec+point_buf.front()->header.stamp.nanosec * (1e-9)) < (pose_msg->header.stamp.sec+pose_msg->header.stamp.nanosec * (1e-9)))
                    point_buf.pop();
                point_msg = point_buf.front();
                point_buf.pop();
            }
        }
        m_buf.unlock();

        if (pose_msg != NULL)
        {
            //printf(" pose time %f \n", pose_msg->header.stamp.toSec());
            //printf(" point time %f \n", point_msg->header.stamp.toSec());
            //printf(" image time %f \n", image_msg->header.stamp.toSec());
            // skip fisrt few
            static int skip_first_cnt = 0;
            if (skip_first_cnt < SKIP_FIRST_CNT)
            {
                skip_first_cnt++;
                continue;
            }


            // limit frequency
            static double last_skip_time = -1;
            if (pose_msg->header.stamp.sec + pose_msg->header.stamp.nanosec * 1e-9  - last_skip_time < SKIP_TIME)
                continue;
            else
                last_skip_time = pose_msg->header.stamp.sec + pose_msg->header.stamp.nanosec * 1e-9;

            // get keyframe pose
            static Eigen::Vector3d last_t(-1e6, -1e6, -1e6);
            Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);
            Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                                     pose_msg->pose.pose.orientation.x,
                                     pose_msg->pose.pose.orientation.y,
                                     pose_msg->pose.pose.orientation.z).toRotationMatrix();

            // add keyframe
            if((T - last_t).norm() > SKIP_DIST)
            {
                // convert image
                cv_bridge::CvImageConstPtr ptr;
                if (image_msg->encoding == "8UC1")
                {
                    sensor_msgs::msg::Image img;
                    img.header = image_msg->header;
                    img.height = image_msg->height;
                    img.width = image_msg->width;
                    img.is_bigendian = image_msg->is_bigendian;
                    img.step = image_msg->step;
                    img.data = image_msg->data;
                    img.encoding = "mono8";
                    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
                }
                else
                    ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);

                cv::Mat image = ptr->image;

                vector<cv::Point3f> point_3d;
                vector<cv::Point2f> point_2d_uv;
                vector<cv::Point2f> point_2d_normal;
                vector<double> point_id;

                for (unsigned int i = 0; i < point_msg->points.size(); i++)
                {
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;
                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = point_msg->channels[i].values[0];
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x = point_msg->channels[i].values[2];
                    p_2d_uv.y = point_msg->channels[i].values[3];
                    p_id = point_msg->channels[i].values[4];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);
                }

                // new keyframe
                static int global_frame_index = 0;
                KeyFrame* keyframe = new KeyFrame(pose_msg->header.stamp.sec + pose_msg->header.stamp.nanosec * 1e-9, global_frame_index,
                                                  T, R,
                                                  image,
                                                  point_3d, point_2d_uv, point_2d_normal, point_id);

                // detect loop
                m_process.lock();
                loopDetector.addKeyFrame(keyframe, 1);
                m_process.unlock();

                loopDetector.visualizeKeyPoses(pose_msg->header.stamp.sec + pose_msg->header.stamp.nanosec * 1e-9);

                global_frame_index++;
                last_t = T;
            }
        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto n = rclcpp::Node::make_shared("pose_graph");

    // read param

    std::string config_file, config_pkg_path;
    n->declare_parameter<std::string>("config_file", "");
    n->get_parameter("config_file", config_file);
    n->declare_parameter<std::string>("config_pkg_path", "");
    n->get_parameter("config_pkg_path", config_pkg_path);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image_topic"]  >> IMAGE_TOPIC;
    fsSettings["loop_closure"] >> LOOP_CLOSURE;
    fsSettings["skip_time"]    >> SKIP_TIME;
    fsSettings["skip_dist"]    >> SKIP_DIST;
    fsSettings["debug_image"]  >> DEBUG_IMAGE;
    fsSettings["match_image_scale"] >> MATCH_IMAGE_SCALE;
    fsSettings["loop_closure"] >> LOOP_CLOSURE;


    if (LOOP_CLOSURE)
    {

        // initialize vocabulary
        string vocabulary_file;
        fsSettings["vocabulary_file"] >> vocabulary_file;
        vocabulary_file = config_pkg_path + vocabulary_file;
        loopDetector.loadVocabulary(vocabulary_file);

        // initialize brief extractor
        string brief_pattern_file;
        fsSettings["brief_pattern_file"] >> brief_pattern_file;
        brief_pattern_file = config_pkg_path + brief_pattern_file;
        briefExtractor = BriefExtractor(brief_pattern_file);

        // initialize camera model
        m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());
    }

    fsSettings.release();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Pose Graph Started.\033[0m");

    auto sub_image = n->create_subscription<sensor_msgs::msg::Image>(IMAGE_TOPIC, rclcpp::QoS(rclcpp::KeepLast(3)).best_effort(), image_callback);
    auto sub_pose = n->create_subscription<nav_msgs::msg::Odometry>("/vins/odometry/keyframe_pose", rclcpp::QoS(rclcpp::KeepLast(3)).best_effort(), pose_callback);
    auto sub_extrinsic = n->create_subscription<nav_msgs::msg::Odometry>("/vins/domoetry/extrinsic/extrinsic", rclcpp::QoS(rclcpp::KeepLast(3)).best_effort(), extrinsic_callback);
    auto sub_point = n->create_subscription<sensor_msgs::msg::PointCloud>("/vins/odometry/keyframe_point", rclcpp::QoS(rclcpp::KeepLast(3)).best_effort(), point_callback);

    pub_match_img = n->create_publisher<sensor_msgs::msg::Image>("/vins/pose_graph/match_image", 3);
    pub_match_msg = n->create_publisher<std_msgs::msg::Float64MultiArray>("/vins/pose_graph/match_frame", 3);
    pub_key_pose = n->create_publisher<visualization_msgs::msg::MarkerArray>("/vins/pose_graph/keyframe_pose", 3);

    std::thread measurement_process;

    measurement_process = std::thread(process);

    rclcpp::spin(n);

    return 0;
}
