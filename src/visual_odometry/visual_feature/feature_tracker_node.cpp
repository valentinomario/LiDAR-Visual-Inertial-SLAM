#include <memory>
#include "feature_tracker.h"
#include "rclcpp/rclcpp.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/point_cloud.hpp>

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


        pub_feature = this->create_publisher<sensor_msgs::msg::PointCloud>(PROJECT_NAME + "/vins/feature/feature", 5);
        pub_feature_1 = this->create_publisher<sensor_msgs::msg::PointCloud>(PROJECT_NAME + "/vins/feature/feature_1", 5);
        pub_match = this->create_publisher<sensor_msgs::msg::Image>(PROJECT_NAME + "/vins/feature/feature_img", 5);
        pub_match_1 = this->create_publisher<sensor_msgs::msg::Image>(PROJECT_NAME + "/vins/feature/feature_img_1", 5);

        pub_restart = this->create_publisher<std_msgs::msg::Bool>(PROJECT_NAME + "/vins/feature/restart", 5);



        nodeInitialized = true;
        RCLCPP_INFO(shared_from_this()->get_logger(), "-----feature tracker's publishers registred-----");
    }

private:


    void callback_two(const sensor_msgs::msg::Image& img0, const sensor_msgs::msg::Image& img1)
    {
        // https://stackoverflow.com/questions/76568459/ros2-synchronizer-register-callback-inside-a-class-problem

        double cur_img_time = rclcpp::Time(img0.header.stamp).seconds();
        // 1 Whether is the first frame
        if(first_image_flag)
        {
            first_image_flag = false;
            first_image_time = cur_img_time;
            last_image_time = cur_img_time;
            return;
        }

        // 2、Detect unstable camera stream
        if (cur_img_time - last_image_time > 1.0 || cur_img_time < last_image_time)
        {
            RCLCPP_WARN(this->get_logger(), "Image discontinuity detected! Resetting feature tracker.");
            first_image_flag = true;
            last_image_time = 0;
            pub_count = 1;
            auto restart_flag = std_msgs::msg::Bool();
            restart_flag.data = true;
            pub_restart->publish(restart_flag);
            return;
        }
        last_image_time = cur_img_time;
        // frequency control
        if (round(1.0 * pub_count / (cur_img_time - first_image_time)) <= FREQ)
        {
            PUB_THIS_FRAME = true;
            // reset the frequency control
            if (abs(1.0 * pub_count / (cur_img_time - first_image_time) - FREQ) < 0.01 * FREQ)
            {
                first_image_time = cur_img_time;
                pub_count = 0;
            }
        }
        else
        {
            PUB_THIS_FRAME = false;
        }


        // 3、Turn 8UC1 into mono8
        cv_bridge::CvImageConstPtr cv_ptr0, cv_ptr1;
        try
        {
            cv_ptr0 = cv_bridge::toCvShare(std::make_shared<sensor_msgs::msg::Image>(img0), sensor_msgs::image_encodings::MONO8);
            cv_ptr1 = cv_bridge::toCvShare(std::make_shared<sensor_msgs::msg::Image>(img1), sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat show_img_0 = cv_ptr0->image;
        cv::Mat show_img_1 = cv_ptr1->image;


        // 4、Process images and extract visual features
        // TODO: put two images together so that the code can be simplified using for loop
        trackerData[0].readImage(cv_ptr0->image, cur_img_time);
        trackerData[1].readImage(cv_ptr1->image, cur_img_time);
        #if SHOW_UNDISTORTION
            trackerData[0].showUndistortion("undistortion_" + std::to_string(i));
        #endif


        // 5、Update features' id
        for (unsigned int i = 0;; i++)
        {
            bool completed = false;
            for (int j = 0; j < NUM_OF_CAM; j++)
                completed |= trackerData[j].updateID(i);
            if (!completed)
                break;
        }

        // 6、Package the information of all visual features
        if (PUB_THIS_FRAME)
        {
            pub_count++;
            auto feature_points_0 = std::make_shared<sensor_msgs::msg::PointCloud>();
            auto feature_points_1 = std::make_shared<sensor_msgs::msg::PointCloud>();

            feature_points_0->header.stamp = img0.header.stamp;
            feature_points_0->header.frame_id = "vins_body";
            feature_points_1->header.stamp = img1.header.stamp;
            feature_points_1->header.frame_id = "vins_body";

            std::vector<std::set<int>> hash_ids(NUM_OF_CAM);
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                sensor_msgs::msg::ChannelFloat32 id_of_point;
                sensor_msgs::msg::ChannelFloat32 u_of_point;
                sensor_msgs::msg::ChannelFloat32 v_of_point;
                sensor_msgs::msg::ChannelFloat32 velocity_x_of_point;
                sensor_msgs::msg::ChannelFloat32 velocity_y_of_point;

                auto &un_pts = trackerData[i].cur_un_pts;
                auto &cur_pts = trackerData[i].cur_pts;
                auto &ids = trackerData[i].ids;
                auto &pts_velocity = trackerData[i].pts_velocity;

                for (unsigned int j = 0; j < ids.size(); j++)
                {
                    if (trackerData[i].track_cnt[j] > 1)
                    {
                        int p_id = ids[j];
                        hash_ids[i].insert(p_id);
                        geometry_msgs::msg::Point32 p;
                        p.x = un_pts[j].x;
                        p.y = un_pts[j].y;
                        p.z = 1;

                        if(i == 0)
                            feature_points_0->points.push_back(p);
                        else
                            feature_points_1->points.push_back(p);
                        // used to identify the feature point belongs to which camera
                        id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                        u_of_point.values.push_back(cur_pts[j].x);
                        v_of_point.values.push_back(cur_pts[j].y);
                        velocity_x_of_point.values.push_back(pts_velocity[j].x);
                        velocity_y_of_point.values.push_back(pts_velocity[j].y);
                    }
                }

                if(i == 0)
                {
                    feature_points_0->channels.push_back(id_of_point);
                    feature_points_0->channels.push_back(u_of_point);
                    feature_points_0->channels.push_back(v_of_point);
                    feature_points_0->channels.push_back(velocity_x_of_point);
                    feature_points_0->channels.push_back(velocity_y_of_point);
                }
                else
                {
                    feature_points_1->channels.push_back(id_of_point);
                    feature_points_1->channels.push_back(u_of_point);
                    feature_points_1->channels.push_back(v_of_point);
                    feature_points_1->channels.push_back(velocity_x_of_point);
                    feature_points_1->channels.push_back(velocity_y_of_point);
                }
            }


            // 7、get features' depths from lidar point cloud
            pcl::PointCloud<PointType>::Ptr depth_cloud_temp(new pcl::PointCloud<PointType>());
            mtx_lidar.lock();
            *depth_cloud_temp = *depthCloud;
            mtx_lidar.unlock();
            // depth_cloud_temp in vins_world
            // feature_points->points  in camera_link
            sensor_msgs::msg::ChannelFloat32 depth_of_points_0 = depthRegister->get_depth(img0.header.stamp, show_img_0, depth_cloud_temp, trackerData[0].m_camera, feature_points_0->points, 0);
            sensor_msgs::msg::ChannelFloat32 depth_of_points_1 = depthRegister->get_depth(img1.header.stamp, show_img_1, depth_cloud_temp, trackerData[1].m_camera, feature_points_1->points, 1);

            feature_points_0->channels.push_back(depth_of_points_0);
            feature_points_1->channels.push_back(depth_of_points_1);

            // skip the first image; since no optical speed on frist image
            if (!init_pub)
            {
                init_pub = 1;
            }
            else
            {
                pub_feature->publish(*feature_points_0);
                pub_feature_1->publish(*feature_points_1);
            }


            // publish features in image
            if (pub_match->get_subscription_count() != 0)
            {
                cv::Mat tmp_img_0;
                cv::Mat tmp_img_1;
                cv_ptr0 = cv_bridge::cvtColor(cv_ptr0, sensor_msgs::image_encodings::RGB8);
                cv_ptr1 = cv_bridge::cvtColor(cv_ptr1, sensor_msgs::image_encodings::RGB8);
                cv::Mat stereo_img_0 = cv_ptr0->image;
                cv::Mat stereo_img_1 = cv_ptr1->image;

                for (int i = 0; i < NUM_OF_CAM; i++)
                {
                    if(i == 0)
                    {
                        tmp_img_0 = stereo_img_0;
                        cv::cvtColor(show_img_0, tmp_img_0, CV_GRAY2RGB);
                    }
                    else
                    {
                        tmp_img_1 = stereo_img_1;
                        cv::cvtColor(show_img_1, tmp_img_1, CV_GRAY2RGB);
                    }

                    for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                    {
                        if (SHOW_TRACK)
                        {
                            // track count
                            double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                            if(i == 0)
                            {
                                cv::circle(tmp_img_0, trackerData[i].cur_pts[j], 4, cv::Scalar(255 * (1 - len), 255 * len, 0), 4);
                            }
                            else
                            {
                                cv::circle(tmp_img_1, trackerData[i].cur_pts[j], 4, cv::Scalar(255 * (1 - len), 255 * len, 0), 4);
                            }

                        }

                    }
                }

                pub_match->publish(*cv_ptr0->toImageMsg());
                pub_match_1->publish(*cv_ptr1->toImageMsg());
            }
        }



        static int visual_cnt = 0;
        // RCLCPP_INFO(this->get_logger(), "Camera callback works TODO remove this log %d", visual_cnt);
        visual_cnt++;

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


        // 1. convert laser cloud message to pcl
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
        // RCLCPP_INFO(this->get_logger(), "Lidar callback works TODO remove this log %d", lidar_cnt);
        lidar_cnt++;

    }

    // cameras handler
    FeatureTracker trackerData[NUM_OF_CAM_ALL]; // TODO: cani

    double first_image_time;
    int pub_count = 1;
    bool first_image_flag = true;
    double last_image_time = 0;
    bool init_pub = 0;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_img0;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_img1;
    std::shared_ptr<message_filters::Synchronizer<ImgSyncPolicy>> sync;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_feature;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_feature_1;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match_1;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_restart;

    // lidar handler
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

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}