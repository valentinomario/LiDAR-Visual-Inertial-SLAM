#include "utility.h"
#include "cloud_msg/msg/cloud_info.hpp"
#include "cloud_msg/msg/custom_point.hpp"
// #include "cloud_msg/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

struct LivoxCustomPointXYZILTD{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint8_t line;
    uint8_t tag;
    uint32_t offset_time;
    float distance;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(LivoxCustomPointXYZILTD,
                                  (float, x, x) (float, y, y) (float, z, z)
                                          (float, distance, distance)
                                          (float, intensity, intensity) (uint8_t, line, line) (uint8_t, tag, tag)
                                          (uint32_t, offset_time, offset_time)
);


// Set LivoxCustomPointXYZILTD as default representation
using PointXYZIRT = LivoxCustomPointXYZILTD;

const int queueLength = 2000;

class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;
    std::mutex odoLock;

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subLaserCloud;
    rclcpp::CallbackGroup::SharedPtr callbackGroupLidar;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExtractedCloud;
    rclcpp::Publisher<cloud_msg::msg::CloudInfo>::SharedPtr pubLaserCloudInfo;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::CallbackGroup::SharedPtr callbackGroupImu;
    std::deque<sensor_msgs::msg::Imu> imuQueue;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
    rclcpp::CallbackGroup::SharedPtr callbackGroupOdom;
    std::deque<nav_msgs::msg::Odometry> odomQueue;

    std::deque<livox_ros_driver2::msg::CustomMsg> cloudQueue;
    livox_ros_driver2::msg::CustomMsg currentCloudMsg;

    std::shared_ptr<ImuTracker> imu_tracker_;
    bool imu_tracker_init = false;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointType>::Ptr   extractedCloud;

    int ringFlag = 0;
    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    cloud_msg::msg::CloudInfo cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::msg::Header cloudHeader;

    vector<int> columnIdnCountVec;


public:
    ImageProjection(const rclcpp::NodeOptions & options) :
            ParamServer("lio_sam_imageProjection", options), deskewFlag(0)
    {
        callbackGroupLidar = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupImu = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupOdom = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        auto lidarOpt = rclcpp::SubscriptionOptions();
        lidarOpt.callback_group = callbackGroupLidar;
        auto imuOpt = rclcpp::SubscriptionOptions();
        imuOpt.callback_group = callbackGroupImu;
        auto odomOpt = rclcpp::SubscriptionOptions();
        odomOpt.callback_group = callbackGroupOdom;

        subImu = create_subscription<sensor_msgs::msg::Imu>(
            imuTopic, qos_imu,
            std::bind(&ImageProjection::imuHandler, this, std::placeholders::_1),
            imuOpt);
        subOdom = create_subscription<nav_msgs::msg::Odometry>(
            odomTopic + "_incremental", qos_imu,
            std::bind(&ImageProjection::odometryHandler, this, std::placeholders::_1),
            odomOpt);
        subLaserCloud = create_subscription<livox_ros_driver2::msg::CustomMsg>(
            pointCloudTopic, qos_lidar,
            std::bind(&ImageProjection::cloudHandler, this, std::placeholders::_1),
            lidarOpt);

        pubExtractedCloud = create_publisher<sensor_msgs::msg::PointCloud2>(
            "lio_sam/deskew/cloud_deskewed", 1);
        pubLaserCloudInfo = create_publisher<cloud_msg::msg::CloudInfo>(
            "lio_sam/deskew/cloud_info", qos);

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(VRES*HRES);

        cloudInfo.start_ring_index.assign(VRES, 0);
        cloudInfo.end_ring_index.assign(VRES, 0);

        cloudInfo.point_col_ind.assign(VRES*HRES, 0);
        cloudInfo.point_range.assign(VRES*HRES, 0);

        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection
        rangeMat = cv::Mat(VRES, HRES, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }
        columnIdnCountVec.assign(VRES, 0);
    }

    ~ImageProjection(){}

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg)
    {
        scaleImuAcceleration(imuMsg);

        if (!imu_tracker_init)
        {
            imu_tracker_init = true;

            imu_tracker_.reset(new ImuTracker(10.0, stamp2Sec(imuMsg->header.stamp)));
        }

        sensor_msgs::msg::Imu thisImu = imuConverter(*imuMsg, imu_tracker_);

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);


        // debug IMU data
        cout << std::setprecision(6);
        cout << "IMU acc: " << endl;
        cout << "x: " << thisImu.linear_acceleration.x <<
              ", y: " << thisImu.linear_acceleration.y <<
              ", z: " << thisImu.linear_acceleration.z << endl;
        cout << "IMU gyro: " << endl;
        cout << "x: " << thisImu.angular_velocity.x <<
              ", y: " << thisImu.angular_velocity.y <<
              ", z: " << thisImu.angular_velocity.z << endl;
        double imuRoll, imuPitch, imuYaw;
        tf2::Quaternion orientation;
        tf2::fromMsg(thisImu.orientation, orientation);
        tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        cout << "IMU roll pitch yaw: " << endl;
        cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;

    }

    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void cloudHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr laserCloudMsg)
    {
        if (!cachePointCloud(laserCloudMsg))
            return;

        if (!deskewInfo())
            return;


        int skippedPoints = 0;
        // Perform downsampling, deskewing and save on rangeMat
        int downSampleId = 0;
        for (auto & point : laserCloudIn->points)
        {
            // downsample
            downSampleId++;
            if(downSampleId % downsampleRate != 0) continue;

            // filter invalid points (LIVOX uses (0 0 0) for invalid points)
            // if (point.x == 0.0 && point.y == 0.0 && point.z == 0.0) continue; // using distance when caching

            // TODO: probabilmente è inutile convertire in PointType, si può fare il
            //       deskewing direttamemte con il LivoxCustomPointXYZILTD

            PointType thisPoint;
            thisPoint.x = point.x;
            thisPoint.y = point.y;
            thisPoint.z = point.z;
            thisPoint.intensity = point.intensity;

            double pointTime = timeScanCur + point.offset_time * 1e-9;
            thisPoint = deskewPoint(&thisPoint, pointTime);

            if(!projectPoint(thisPoint, point.distance)){
                skippedPoints++;
                //continue;
            }


        }

        cloudExtraction();

        publishClouds();

        resetParameters();
    }

    bool cachePointCloud(const livox_ros_driver2::msg::CustomMsg::SharedPtr& laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2)
            return false;

        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();

        // Probably the following lines can be replaced by pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);  TODO: check!
        laserCloudIn->clear();
        laserCloudIn->reserve(currentCloudMsg.point_num);

        for (size_t i = 0; i < laserCloudMsg->point_num; i++)
        {
            const livox_ros_driver2::msg::CustomPoint& src = laserCloudMsg->points[i];
            PointXYZIRT dst;

            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.intensity = src.reflectivity;
            dst.offset_time = src.offset_time;
            dst.tag = src.tag;
            dst.line = src.line;
            dst.offset_time = src.offset_time;
            dst.distance = sqrt(src.x * src.x + src.y * src.y + src.z * src.z);
            if (dst.distance != 0.0)
                laserCloudIn->push_back(dst);

        }
        laserCloudIn->is_dense = true;

        // get timestamp
        cloudHeader = currentCloudMsg.header;
        timeScanCur = stamp2Sec(cloudHeader.stamp);
        timeScanEnd = timeScanCur + laserCloudIn->points.back().offset_time * 1e-9;

        // TODO probably useless in our case
        // remove Nan
        vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);

        // check dense flag
        if (laserCloudIn->is_dense == false)
        {
            RCLCPP_ERROR(get_logger(), "Point cloud is not in dense format, please remove NaN points first!");
            rclcpp::shutdown();
        }


        return true;
    }

    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure IMU data available for the scan
        if (imuQueue.empty() ||
            stamp2Sec(imuQueue.front().header.stamp) > timeScanCur ||
            stamp2Sec(imuQueue.back().header.stamp) < timeScanEnd)
        {
            // RCLCPP_INFO(get_logger(), "Waiting for IMU data ...");
            return false;
        }

        imuDeskewInfo();

        odomDeskewInfo();

        return true;
    }

    void imuDeskewInfo()
    {
        cloudInfo.imu_available = false;

        while (!imuQueue.empty())
        {
            if (stamp2Sec(imuQueue.front().header.stamp) < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::msg::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = stamp2Sec(thisImuMsg.header.stamp);

            // get roll, pitch, and yaw estimation for this scan
            if (currentImuTime <= timeScanCur)
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imu_roll_init, &cloudInfo.imu_pitch_init, &cloudInfo.imu_yaw_init);
            if (currentImuTime > timeScanEnd + 0.01)
                break;

            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        cloudInfo.imu_available = true;
    }

    void odomDeskewInfo()
    {
        cloudInfo.odom_available = false;

        while (!odomQueue.empty())
        {
            if (stamp2Sec(odomQueue.front().header.stamp) < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (stamp2Sec(odomQueue.front().header.stamp) > timeScanCur)
            return;

        // get start odometry at the beinning of the scan
        nav_msgs::msg::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (stamp2Sec(startOdomMsg.header.stamp) < timeScanCur)
                continue;
            else
                break;
        }

        tf2::Quaternion orientation;
        tf2::fromMsg(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess used in mapOptimization
        cloudInfo.odom_x = startOdomMsg.pose.pose.position.x;
        cloudInfo.odom_y = startOdomMsg.pose.pose.position.y;
        cloudInfo.odom_z = startOdomMsg.pose.pose.position.z;
        cloudInfo.odom_roll = roll;
        cloudInfo.odom_pitch = pitch;
        cloudInfo.odom_yaw = yaw;

        cloudInfo.odom_available = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (stamp2Sec(odomQueue.back().header.stamp) < timeScanEnd)
            return;

        nav_msgs::msg::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (stamp2Sec(endOdomMsg.header.stamp) < timeScanEnd)
                continue;
            else
                break;
        }

        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf2::fromMsg(endOdomMsg.pose.pose.orientation, orientation);
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanEnd - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
    }

    PointType deskewPoint(PointType *point, double pointTime)
    {
        if (deskewFlag == -1 || cloudInfo.imu_available == false)
            return *point;

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(pointTime - timeScanCur, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    bool projectPoint(PointType& point, float dist){ // TODO usare il LivoxCustomPointXYZILTD
        float phi = atan2(point.y, point.x) * 180.0 / M_PI; // horizontal
        float theta = acos(point.z / dist) * 180.0 / M_PI; // vertical, small theta corresponding to upper points

        // Get row index based on theta
        // float thetaSpan = fovMaxTheta - fovMinTheta;
        float thetaSpan = fovMaxTheta - fovMinTheta;
        int row = round((fovMaxTheta - theta) / thetaSpan * (VRES - 1));
        if(row>=VRES) row = VRES-1;
        else if(row<0) row = 0;

        static float angularResolution = 360.0/float(HRES);
        int column = -round((phi-90.0)/angularResolution) + HRES/2;
        if (column >= HRES)
            column -= HRES;

        if(column<0) column = 0;

        if (rangeMat.at<float>(row, column) != FLT_MAX)
            return false;

        rangeMat.at<float>(row, column) = dist;

        int index = column  + row * HRES;
        fullCloud->points[index] = point;
        return true;
    }

    void cloudExtraction()
    {
        // This function extracts the point cloud that composes rangeMat and starts generating the cloud_info fields
        int count = 0;

        for (int i = 0; i < VRES; ++i)
        {
            cloudInfo.start_ring_index[i] = count - 1 + 5;

            for (int j = 0; j < HRES; ++j)
            {
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {
                    // mark the points' column index for marking occlusion later
                    cloudInfo.point_col_ind[count] = j;
                    // save range info
                    cloudInfo.point_range[count] = rangeMat.at<float>(i,j);
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i*HRES]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.end_ring_index[i] = count -1 - 5;
        }
    }

    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed  = publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo->publish(cloudInfo);
    }
};

int main(int argc, char** argv)
{
    // args: --ros-args --params-file /home/user/ros2_ws/install/lio_sam/share/lio_sam/config/params.yaml
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor exec;

    auto IP = std::make_shared<ImageProjection>(options);
    exec.add_node(IP);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Image Projection Started.\033[0m");

    exec.spin();

    rclcpp::shutdown();
    return 0;
}