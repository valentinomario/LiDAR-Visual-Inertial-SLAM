#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::msg::Image::ConstPtr> img_buf;

rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_feature;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_restart;

FeatureTracker trackerData[NUM_OF_CAM];
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

std::mutex mtx_lidar;

pcl::PointCloud<PointType>::Ptr depthCloud(new pcl::PointCloud<PointType>());

deque<pcl::PointCloud<PointType>> cloudQueue;
deque<double> timeQueue;

DepthRegister *depthRegister;

std::shared_ptr<tf2_ros::Buffer> tfBuffer;
std::shared_ptr<tf2_ros::TransformListener> listener;
tf2::Stamped<tf2::Transform> tfTransform;

void img_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    double cur_img_time = img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9);

    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = cur_img_time;
        last_image_time = cur_img_time;
        return;
    }

    // detect unstable camera stream
    if (cur_img_time - last_image_time > 1.0 || cur_img_time < last_image_time)
    {
        RCUTILS_LOG_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::msg::Bool restart_flag;
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
        PUB_THIS_FRAME = false;

    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::msg::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image;
    // cv::imshow("img", show_img);
    // cv::waitKey(0);
    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        RCUTILS_LOG_DEBUG("processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), img_msg->header.stamp.sec+img_msg->header.stamp.nanosec * (1e-9));
        else
        {
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

   if (PUB_THIS_FRAME)
   {
        pub_count++;
        sensor_msgs::msg::PointCloud::SharedPtr feature_points(new sensor_msgs::msg::PointCloud);
        sensor_msgs::msg::ChannelFloat32 id_of_point;
        sensor_msgs::msg::ChannelFloat32 u_of_point;
        sensor_msgs::msg::ChannelFloat32 v_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_y_of_point;

        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "vins_body"; // TODO era world

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
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

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }

        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);

        // RCUTILS_LOG_DEBUG("publish %f, at %f", feature_points->header.stamp.sec+feature_points->header.stamp.nanosec * (1e-9), rclcpp::Clock().now().nanoseconds()*(1e-9));

       // get feature depth from lidar point cloud
        pcl::PointCloud<PointType>::Ptr depth_cloud_temp(new pcl::PointCloud<PointType>());
        mtx_lidar.lock();
        *depth_cloud_temp = *depthCloud;
        mtx_lidar.unlock();

        sensor_msgs::msg::ChannelFloat32 depth_of_points = depthRegister->get_depth(img_msg->header.stamp, show_img, depth_cloud_temp, trackerData[0].m_camera, feature_points->points);
        feature_points->channels.push_back(depth_of_points);

        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_feature->publish(*feature_points);

        if (pub_match->get_subscription_count() != 0)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::RGB8); // TODO check encoding
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    if (SHOW_TRACK)
                    {
                        // track count
                        double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                        cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(255 * (1 - len), 255 * len, 0), 4);
                    } else {
                        // depth
                        if(j < depth_of_points.values.size())
                        {
                            if (depth_of_points.values[j] > 0)
                                cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(0, 255, 0), 4);
                            else
                                cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(0, 0, 255), 4);
                        }
                    }
                }
            }
            //cv::imshow("vis", stereo_img);
            //cv::waitKey(5);
            pub_match->publish(*(ptr->toImageMsg()));
        }
    }
    // RCUTILS_LOG_INFO("whole feature tracker processing costs: %fms", t_r.toc());
}

/* TODO: remove, we don't use the cloud coming from the lidar here but from a LIO node
void moveFromCustomMsg(const livox_ros_driver2::msg::CustomMsg::SharedPtr& laserCloudMsg, pcl::PointCloud<PointType>& outCloud )
{
    outCloud.clear();
    outCloud.reserve(laserCloudMsg->point_num);
    PointType point;

    outCloud.header.frame_id=laserCloudMsg->header.frame_id;
    outCloud.header.stamp = (uint64_t)((laserCloudMsg->header.stamp.sec*1e9 + laserCloudMsg->header.stamp.nanosec)/1000) ;
    // cloud.header.seq=Msg.header.seq;

    for(uint i=0;i<laserCloudMsg->point_num-1;i++)
    {
        point.x=laserCloudMsg->points[i].x;
        point.y=laserCloudMsg->points[i].y;
        point.z=laserCloudMsg->points[i].z;
        point.intensity=laserCloudMsg->points[i].reflectivity;
        //point.tag=Msg.points[i].tag;
        // point.time=Msg.points[i].offset_time*1e-9;
        // point.ring=Msg.points[i].line;
        outCloud.push_back(point);
    }
}
*/
void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr laser_msg)
{
    static int lidar_count = -1;
    if (++lidar_count % (LIDAR_SKIP+1) != 0)
        return;

    // 0. listen to transform
    try{
        tf2::fromMsg(
            tfBuffer->lookupTransform(
                "vins_world",
                "vins_body_ros",
                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
            tfTransform);

        // listener.waitForTransform("vins_world", "vins_body_ros", laser_msg->header.stamp, ros::Duration(0.01));
        // listener.lookupTransform("vins_world", "vins_body_ros", laser_msg->header.stamp, tfTransform);
    }
    catch (tf2::TransformException& ex){
        // ROS_ERROR("lidar no tf");
        return;
    }

    double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
    xCur = tfTransform.getOrigin().x();
    yCur = tfTransform.getOrigin().y();
    zCur = tfTransform.getOrigin().z();
    tf2::Matrix3x3 m(tfTransform.getRotation());
    m.getRPY(rollCur, pitchCur, yawCur);
    Eigen::Affine3f transNow = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);

    // 1. convert laser cloud message to pcl
    pcl::PointCloud<PointType>::Ptr laser_cloud_in(new pcl::PointCloud<PointType>());

    pcl::fromROSMsg(*laser_msg, *laser_cloud_in);
    // moveFromCustomMsg(laser_msg,*laser_cloud_in);

    // 2. downsample new cloud (save memory)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_ds(new pcl::PointCloud<PointType>());
    static pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(laser_cloud_in);
    downSizeFilter.filter(*laser_cloud_in_ds);
    *laser_cloud_in = *laser_cloud_in_ds;

    // 3. filter lidar points (only keep points in camera view)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_filter(new pcl::PointCloud<PointType>());
    for (int i = 0; i < (int)laser_cloud_in->size(); ++i)
    {
        PointType p = laser_cloud_in->points[i];
        if (p.x >= 0 && abs(p.y / p.x) <= 10 && abs(p.z / p.x) <= 10)
            laser_cloud_in_filter->push_back(p);
    }
    *laser_cloud_in = *laser_cloud_in_filter;

    // TODO: transform to IMU body frame
    // 4. offset T_lidar -> T_camera
    pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());
    Eigen::Affine3f transOffset = pcl::getTransformation(L_C_TX, L_C_TY, L_C_TZ, L_C_RX, L_C_RY, L_C_RZ);
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_offset, transOffset);
    *laser_cloud_in = *laser_cloud_offset;

    // 5. transform new cloud into global odom frame
    pcl::PointCloud<PointType>::Ptr laser_cloud_global(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_global, transNow);

    // 6. save new cloud
    double timeScanCur = laser_msg->header.stamp.sec + laser_msg->header.stamp.nanosec*1e-9;
    cloudQueue.push_back(*laser_cloud_global);
    timeQueue.push_back(timeScanCur);

    // 7. pop old cloud
    while (!timeQueue.empty())
    {
        if (timeScanCur - timeQueue.front() > 5.0)
        {
            cloudQueue.pop_front();
            timeQueue.pop_front();
        } else {
            break;
        }
    }

    std::lock_guard<std::mutex> lock(mtx_lidar);
    // 8. fuse global cloud
    depthCloud->clear();
    for (int i = 0; i < (int)cloudQueue.size(); ++i)
        *depthCloud += cloudQueue[i];

    // 9. downsample global cloud
    pcl::PointCloud<PointType>::Ptr depthCloudDS(new pcl::PointCloud<PointType>());
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(depthCloud);
    downSizeFilter.filter(*depthCloudDS);
    *depthCloud = *depthCloudDS;
    cout<<"Lidar cloud size: "<<*depthCloud<<endl;
}

int main(int argc, char **argv)
{
    // --ros-args -r __ns:=/feature_tracker -r __node:=feature_tracker -p config_file:="/home/user/new_ws/install/config_pkg/share/config_pkg/config/euroc/euroc_config.yaml" -p vins_folder:="/home/user/new_ws/install/config_pkg/share/config_pkg/config/../"
    rclcpp::init(argc, argv);
    auto n = rclcpp::Node::make_shared("feature_tracker");

    readParameters(n);

    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                RCUTILS_LOG_INFO("load mask fail");
            }
            else
                RCUTILS_LOG_INFO("load mask success");
        }
    }

    depthRegister = new DepthRegister(n);

    tfBuffer = std::make_shared<tf2_ros::Buffer>(n->get_clock());
    listener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
    auto sub_img = n->create_subscription<sensor_msgs::msg::Image>(IMAGE_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)), img_callback);
    auto sub_lidar = n->create_subscription<sensor_msgs::msg::PointCloud2>(POINT_CLOUD_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)), lidar_callback);

    pub_feature = n->create_publisher<sensor_msgs::msg::PointCloud>( + "/vins/feature/feature", 1000);
    pub_match = n->create_publisher<sensor_msgs::msg::Image>(PROJECT_NAME + "/vins/feature/feature_img",1000);
    pub_restart = n->create_publisher<std_msgs::msg::Bool>(PROJECT_NAME + "/vins/feature/restart",1000);
    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */
    rclcpp::spin(n);
    return 0;
}


// new points velocity is 0, pub or not?
// track cnt > 1 pub?