#include <memory>
#include "feature_tracker.h"
#include "rclcpp/rclcpp.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

std::mutex mtx_lidar;
pcl::PointCloud<PointType>::Ptr depthCloud(new pcl::PointCloud<PointType>());
std::deque<pcl::PointCloud<PointType>> cloudQueue;
std::deque<double> timeQueue;
DepthRegister *depthRegister;

using ImgSyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image>;

class FeatureTrackerNode : public rclcpp::Node
{
public:
    FeatureTrackerNode() : Node("feature_tracker_node")
    {
        RCLCPP_INFO(this->get_logger(), "\033[1;32mVisual Feature Tracker Started.\033[0m");

        // Lidars parameters
        this->declare_parameter<std::string>("PROJECT_NAME", "emv_lio2");
        this->get_parameter("PROJECT_NAME", PROJECT_NAME);

        auto extRotV_lidar2imu = this->declare_parameter<std::vector<double>>("extrinsicRot", {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0});
        auto extTransV_lidar2imu = this->declare_parameter<std::vector<double>>("extrinsicTrans", {0.0,0.0,0.0});
        extRot_lidar2imu = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV_lidar2imu.data(), 3, 3);
        extTrans_lidar2imu = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV_lidar2imu.data(), 3, 1);

        NUM_OF_CAM = this->declare_parameter<int>("NUM_OF_CAM", 1);

        // Load config files
        // std::string pkg_path = ament_index_cpp::get_package_share_directory(PROJECT_NAME);
        auto config_path = this->declare_parameter<std::string>("config_dir", "/home/user/ros2_ws/install/emv_lio2/share/emv_lio2/config");

        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        readParameters(config_path);
    }

    void initPubSub()
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);
        }

        if(FISHEYE)
        {
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
                if(!trackerData[i].fisheye_mask.data)
                {
                    RCLCPP_ERROR(shared_from_this()->get_logger(), "load fisheye mask fail");
                }
                else
                    RCLCPP_INFO(shared_from_this()->get_logger(),"load mask success");
            }
        }

        depthRegister = new DepthRegister(shared_from_this());

        if(NUM_OF_CAM == 2)
        {
            sub_img0 = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
                shared_from_this(), IMAGE_TOPIC_0);
            sub_img1 = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
                shared_from_this(), IMAGE_TOPIC_1);

            sync = std::make_shared<message_filters::Synchronizer<ImgSyncPolicy>>(
                    ImgSyncPolicy(200), *sub_img0, *sub_img1);

            //sync->registerCallback(std::bind(&FeatureTrackerNode::callback_two, this, std::placeholders::_1, std::placeholders::_2));
            sync->registerCallback(&FeatureTrackerNode::callback_two, this);
        }

        sub_lidar = this->create_subscription<sensor_msgs::msg::PointCloud2>(POINT_CLOUD_TOPIC, 5,
            std::bind(&FeatureTrackerNode::lidar_callback, this, std::placeholders::_1));





        nodeInitialized = true;
        RCLCPP_INFO(shared_from_this()->get_logger(), "-----------Node initialized-------- ");
    }

private:


    void callback_two(const sensor_msgs::msg::Image& img0, const sensor_msgs::msg::Image& img1)
    {
        // https://stackoverflow.com/questions/76568459/ros2-synchronizer-register-callback-inside-a-class-problem

        static int i = 0;
        RCLCPP_INFO(this->get_logger(), "Lidar callback works TODO remove this log %d", i);
        i++;

    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2& laser_msg)
    {
        static int lidar_count = -1;
        if (++lidar_count % (LIDAR_SKIP+1) != 0)
            return;

        // 0. listen to transform

        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            transformStamped = tf_buffer->lookupTransform(
                       "vins_world", "vins_body_ros", laser_msg.header.stamp, tf2::durationFromSec(0.01));
        }
        catch (tf2::TransformException ex){
            // ROS_ERROR("lidar no tf");
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        }

        Eigen::Affine3f transNow = tf2::transformToEigen(transformStamped).cast<float>();


        // 1. convert laser cloud message to pcl，此时laser_cloud_in是pcl格式的/hesai/pandar点云(360°的)
        pcl::PointCloud<PointType>::Ptr laser_cloud_in(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(laser_msg, *laser_cloud_in);

        // 2. downsample current cloud (save memory)
        pcl::PointCloud<PointType>::Ptr laser_cloud_in_ds(new pcl::PointCloud<PointType>());

        down_size_random_sample_filter.setInputCloud(laser_cloud_in);
        down_size_random_sample_filter.setSample(laser_cloud_in->size() * 0.3);
        down_size_random_sample_filter.filter(*laser_cloud_in_ds);

        *laser_cloud_in = *laser_cloud_in_ds;


        // TODO: transform to IMU body frame
        // 4. offset T_lidar -> T_camera
        pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());
        Eigen::Affine3f transOffset = pcl::getTransformation(0,
                                                             0,
                                                             0,
                                                             0,
                                                             0,
                                                             0);  // lidar to vins_body_ros

        pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_offset, transOffset);
        *laser_cloud_in = *laser_cloud_offset;

        // 5. transform new cloud into global odom frame
        pcl::PointCloud<PointType>::Ptr laser_cloud_global(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_global, transNow);

        // 6. save new cloud
        double timeScanCur = rclcpp::Time(laser_msg.header.stamp).seconds();
        cloudQueue.push_back(*laser_cloud_global);
        timeQueue.push_back(timeScanCur);

        // 7. pop old cloud
        while (!timeQueue.empty())
        {
            if (timeScanCur - timeQueue.front() > 2.0)
            {
                cloudQueue.pop_front();
                timeQueue.pop_front();
            }
            else
            {
                break;
            }
        }

        std::lock_guard<std::mutex> lock(mtx_lidar);
        // 8. fuse global cloud
        depthCloud->clear();
        for (int i = 0; i < static_cast<int>(cloudQueue.size()); ++i)
        {
            *depthCloud += cloudQueue[i];
        }


        // 9. downsample global cloud
        pcl::PointCloud<PointType>::Ptr depthCloudDS(new pcl::PointCloud<PointType>());
        down_size_random_sample_filter.setInputCloud(depthCloud);
        down_size_random_sample_filter.setSample(depthCloud->size() * 0.3);
        down_size_random_sample_filter.filter(*depthCloudDS);

        *depthCloud = *depthCloudDS;

        static int lidar_cnt = 0;
        RCLCPP_INFO(this->get_logger(), "Lidar callback works TODO remove this log %d", lidar_cnt);
        lidar_cnt++;

    }


    FeatureTracker trackerData[NUM_OF_CAM_ALL]; // TODO: cani

    double first_image_time;
    int pub_count = 1;
    bool first_image_flag = true;
    double last_image_time = 0;
    bool init_pub = 0;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_img0;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_img1;
    std::shared_ptr<message_filters::Synchronizer<ImgSyncPolicy>> sync;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar;


    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    pcl::RandomSample<PointType> down_size_random_sample_filter;

    bool nodeInitialized = false;

};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeatureTrackerNode>();
    node->initPubSub();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}