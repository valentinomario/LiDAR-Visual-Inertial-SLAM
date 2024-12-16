#include "utility.h"
#include "cloud_msg/msg/cloud_info.hpp"
#include "cloud_msg/msg/custom_point.hpp"
#include "cloud_msg/msg/custom_msg.hpp"

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

class ScanInfo
{
public:
    ScanInfo(const int &n_scan, const bool &segment_flag)
    {
        segment_flag_ = segment_flag;
        scan_start_ind_.resize(n_scan);
        scan_end_ind_.resize(n_scan);
        ground_flag_.clear();
    }

    std::vector<int> scan_start_ind_, scan_end_ind_;
    bool segment_flag_;
    std::vector<bool> ground_flag_;
};

class ImageProjection : public rclcpp::Node, public ParamServer
{
public:
    ImageProjection() : rclcpp::Node("imageProjection"), deskewFlag(0)
    {
        RCLCPP_INFO(this->get_logger(), "\033[1;32mImage Projection Started.\033[0m");

    }
    void initNode()
    {
        declareParameters(shared_from_this());

        getParameters(shared_from_this());

        pubExtractedCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(PROJECT_NAME + "/lidar/deskew/cloud_deskewed", 5);
        pubLaserCloudInfo = this->create_publisher<cloud_msg::msg::CloudInfo>(PROJECT_NAME + "/lidar/deskew/cloud_info", 5);

        subImu = this->create_subscription<sensor_msgs::msg::Imu>(imuTopic, 2000,
            std::bind(&ImageProjection::imuHandler, this, std::placeholders::_1));
        subOdom = this->create_subscription<nav_msgs::msg::Odometry>(PROJECT_NAME + "/vins/odometry/imu_propagate_ros", 2000,
            std::bind(&ImageProjection::odometryHandler, this, std::placeholders::_1));
        subLaserCloud = this->create_subscription<cloud_msg::msg::CustomMsg>(pointCloudTopic, 5,
            std::bind(&ImageProjection::cloudHandler, this, std::placeholders::_1));

        allocateMemory();

        RCLCPP_INFO(this->get_logger(), "-----image projection's publishers registred-----");
    }

private:

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        //tmpLivoxCloudIn.reset(new pcl::PointCloud<LivoxCustomPointXYZILTD>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(HRES*VRES);

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

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        rangeMat = cv::Mat(VRES,HRES,CV_32F,cv::Scalar::all(FLT_MAX));

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }
    }

    void imuHandler(const std::shared_ptr<sensor_msgs::msg::Imu> imuMsg)
    {
        // This function accumulates the IMU data in the queue

        // Initialize IMU tracker only after the first IMU message arrives
            if (!initialised_)
        {
            initialised_ = true;
            imu_tracker_.reset(new ImuTracker(10.0, ROS_TIME(imuMsg)));
        }

        // Align and track IMU measurement with LiDAR reference frame
        sensor_msgs::msg::Imu thisImu = imuConverter(*imuMsg, imu_tracker_);

        // Push tracked IMU data to imuQueue
        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);
    }

    void odometryHandler(const std::shared_ptr<nav_msgs::msg::Odometry> odomMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odomMsg);
    }

    void cloudHandler(const std::shared_ptr<cloud_msg::msg::CustomMsg> laserCloudMsg){

        // RCLCPP_INFO(this->get_logger(), "cloud points %d", laserCloudMsg->point_num);
        if (!cachePointCloud(laserCloudMsg, laserCloudIn))
            return;

        if (!deskewInfo())
            return;

        // TODO: Prima di questo c'è un if che è sempre falso in cui fanno la segmentazione
        //       in effetti cercando label_mat nel codice, si vede che i dati che verrebbero ottenuti con la segmentazione
        //       non sono mai utilizzati

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
            //       deskewing diretta,emte con il LivoxCustomPointXYZILTD

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

        auto skipRate = (float)skippedPoints/(float)laserCloudMsg->point_num;
        if(skipRate>0.2)
            RCLCPP_WARN(this->get_logger(), "Range matrix resolution HxV: %dx%d is too low! lost %f%% of points",HRES, VRES, skipRate*100);


        cloudExtraction();

        publishClouds();

        resetParameters();
    }

    bool cachePointCloud(const cloud_msg::msg::CustomMsg::SharedPtr laserCloudMsg,
                     pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn)
    {
        // This function adds the received point clouds to the queue, it waits until the queue has at least 3 elements
        // before proceeding, just to work on enough LiDAR points

        // The point cloud is downsampled and filtered

        cloudQueue.push_back(*laserCloudMsg);

        if (cloudQueue.size() <= 2)
            return false;

        currentCloudMsg = cloudQueue.front();
        cloudQueue.pop_front();

        cloudHeader = currentCloudMsg.header;
        timeScanCur = ROS_TIME(&currentCloudMsg);
        timeScanNext = ROS_TIME(&cloudQueue.front());

        laserCloudIn->clear();
        laserCloudIn->reserve(currentCloudMsg.point_num);

        for (size_t i = 0; i < laserCloudMsg->point_num; i++)
        {
            const cloud_msg::msg::CustomPoint& src = laserCloudMsg->points[i];
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

        return true;
    }

    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure IMU data available for the scan
        if (imuQueue.empty() || ROS_TIME(&imuQueue.front()) > timeScanCur || ROS_TIME(&imuQueue.back()) < timeScanNext)
        {
            // ("Waiting for IMU data ...");
            return false;
        }

        getImuDeskewInfo();

        // odomDeskewInfo();        // TODO

        return true;
    }

    void getImuDeskewInfo()
    {
        cloudInfo.imu_available = false;

        while (!imuQueue.empty())
        {
            // Remove old IMU data
            if (ROS_TIME(&imuQueue.front()) < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        for (auto thisImuMsg : imuQueue)
        {
            double currentImuTime = ROS_TIME(&thisImuMsg);

            // Get the initial roll, pitch, and yaw estimation at the beginning of this scan
            // Also add the estimation to cloud_info message for later use in map optimization
            if (currentImuTime <= timeScanCur)
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imu_roll_init, &cloudInfo.imu_pitch_init, &cloudInfo.imu_yaw_init);

            if (currentImuTime > timeScanNext + 0.01)
                break;

            // Set reference as the first element
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

            // integrate rotation (gets the pose for all the time instants)
            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;
        // Check if any de-skewing info was obtained
        if (imuPointerCur <= 0)
            return;

        cloudInfo.imu_available = true;
    }

    void odomDeskewInfo()
    {
        cloudInfo.odom_available = false;

        while (!odomQueue.empty())
        {
            if (ROS_TIME(&odomQueue.front()) < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (ROS_TIME(&odomQueue.front()) > timeScanCur)
            return;

        // get start odometry at the beinning of the scan
        nav_msgs::msg::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
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
        cloudInfo.odom_roll  = roll;
        cloudInfo.odom_pitch = pitch;
        cloudInfo.odom_yaw   = yaw;
        cloudInfo.odom_reset_id = (int)round(startOdomMsg.pose.covariance[0]);

        cloudInfo.odom_available = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (ROS_TIME(&odomQueue.back())< timeScanNext)
            return;

        nav_msgs::msg::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanNext)
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

    PointType deskewPoint(PointType *point, double pointTime)
    {
        if (deskewFlag == -1 || cloudInfo.imu_available == false)
            return *point;

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(pointTime - timeScanCur, &posXCur, &posYCur, &posZCur);

        // Set reference to the first point
        if (firstPointFlag)
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

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        // Get the imu element corresponding to the current time
        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        // If pointTime is after the last element, or before the first, assign the closest corresponding element
        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = (float)imuRotX[imuPointerFront];
            *rotYCur = (float)imuRotY[imuPointerFront];
            *rotZCur = (float)imuRotZ[imuPointerFront];
        } else {
            // Perform linear interpolation between adjacent points
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = (float)(imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack);
            *rotYCur = (float)(imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack);
            *rotZCur = (float)(imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack);
        }
    }

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur) {
        *posXCur = 0;
        *posYCur = 0;
        *posZCur = 0;
    }

    bool projectPoint(PointType& point, float dist){ // TODO usare il LivoxCustomPointXYZILTD
        float phi = atan2(point.y, point.x) * 180.0 / M_PI; // horizontal
        float theta = acos(point.z / dist) * 180.0 / M_PI; // vertical, small theta corresponding to upper points

        // Get row index based on theta
        // float thetaSpan = fovMaxTheta - fovMinTheta;
        float thetaSpan = fov_max_theta - fov_min_theta;
        int row = round((fov_max_theta - theta) / thetaSpan * (VRES - 1));
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
        cloudInfo.cloud_deskewed  = publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp, "base_link");
        pubLaserCloudInfo->publish(cloudInfo);
    }

    const int queueLength = 1000;

    std::shared_ptr<ImuTracker> imu_tracker_;
    bool initialised_ = false;

    std::mutex imuLock;
    std::mutex odoLock;

    rclcpp::Publisher<cloud_msg::msg::CloudInfo>::SharedPtr pubLaserCloudInfo;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExtractedCloud;

    rclcpp::Subscription<cloud_msg::msg::CustomMsg>::SharedPtr subLaserCloud;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;

    std::deque<sensor_msgs::msg::Imu> imuQueue;

    std::deque<nav_msgs::msg::Odometry> odomQueue;

    std::deque<cloud_msg::msg::CustomMsg> cloudQueue;
    cloud_msg::msg::CustomMsg currentCloudMsg;

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

    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    cloud_msg::msg::CloudInfo cloudInfo;

    double timeScanCur;
    double timeScanNext;
    std_msgs::msg::Header cloudHeader;

    Eigen::MatrixXf range_mat;
};


int main(int argc, char **argv) {
    // args: --ros-args --params-file /home/user/ros2_ws/install/emv_lio2/share/emv_lio2/config/params_lidar.yaml -r __node:=imageProjection
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageProjection>();
    node->initNode();

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 3);
    exec.add_node(node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}