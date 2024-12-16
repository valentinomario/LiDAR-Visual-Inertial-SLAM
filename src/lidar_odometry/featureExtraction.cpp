#include "utility.h"
#include "cloud_msg/msg/cloud_info.hpp"

struct smoothness_t{
    float value;
    size_t ind;
};

struct by_value{
    bool operator()(smoothness_t const &left, smoothness_t const &right) {
        return left.value < right.value;
    }
};

class FeatureExtraction : public rclcpp::Node, public ParamServer
{
public:
    FeatureExtraction() : Node("featureExtraction")
    {
        RCLCPP_INFO(this->get_logger(), "\033[1;32m Feature Extraction started\033[0m");


    }

    void initNode()
    {
        declareParameters(shared_from_this());
        getParameters(shared_from_this());

        subLaserCloudInfo = this->create_subscription<cloud_msg::msg::CloudInfo>(PROJECT_NAME + "/lidar/deskew/cloud_info", 5,
            std::bind(&FeatureExtraction::laserCloudInfoHandler, this, std::placeholders::_1));

        pubLaserCloudInfo = this->create_publisher<cloud_msg::msg::CloudInfo>(PROJECT_NAME + "/lidar/feature/cloud_info", 5);
        pubCornerPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(PROJECT_NAME + "/lidar/feature/cloud_corner", 5);
        pubSurfacePoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(PROJECT_NAME + "/lidar/feature/cloud_surface", 5);

        cloudSmoothness.resize(VRES*HRES);

        downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

        extractedCloud.reset(new pcl::PointCloud<PointType>());
        cornerCloud.reset(new pcl::PointCloud<PointType>());
        surfaceCloud.reset(new pcl::PointCloud<PointType>());

        cloudCurvature = new float[VRES*HRES];
        cloudNeighborPicked = new int[VRES*HRES];
        cloudLabel = new int[VRES*HRES];

        RCLCPP_INFO(this->get_logger(), "-----feature extraction's publishers initialized-----");

    }

private:

    void laserCloudInfoHandler(cloud_msg::msg::CloudInfo::SharedPtr msgIn)
    {
        cloudInfo = *msgIn; // new cloud info
        cloudHeader = msgIn->header; // new cloud header
        pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud); // new cloud for extraction

        if (feature_enable)
        {
            /*
            calculateSmoothness();

            markOccludedPoints();

            extractFeatures();
            */
            RCLCPP_ERROR(this->get_logger(), "lidar feature extraction enabled but not implemented!");
        }
        else
        {
            pcl::copyPointCloud(*extractedCloud, *surfaceCloud);
        }

        publishFeatureCloud();
    }

    void publishFeatureCloud()
    {
        // free cloud info memory
        freeCloudInfoMemory();
        // save newly extracted features
        cloudInfo.cloud_corner  = publishCloud(pubCornerPoints,  cornerCloud,  cloudHeader.stamp, "base_link");
        cloudInfo.cloud_surface = publishCloud(pubSurfacePoints, surfaceCloud, cloudHeader.stamp, "base_link");
        // publish to mapOptimization
        pubLaserCloudInfo->publish(cloudInfo);
    }

    void freeCloudInfoMemory()
    {
        cloudInfo.start_ring_index.clear();
        cloudInfo.start_ring_index.shrink_to_fit();
        cloudInfo.end_ring_index.clear();
        cloudInfo.end_ring_index.shrink_to_fit();
        cloudInfo.point_col_ind.clear();
        cloudInfo.point_col_ind.shrink_to_fit();
        cloudInfo.point_range.clear();
        cloudInfo.point_range.shrink_to_fit();
    }

    rclcpp::Subscription<cloud_msg::msg::CloudInfo>::SharedPtr subLaserCloudInfo;

    rclcpp::Publisher<cloud_msg::msg::CloudInfo>::SharedPtr pubLaserCloudInfo;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPoints;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfacePoints;

    pcl::PointCloud<PointType>::Ptr extractedCloud;
    pcl::PointCloud<PointType>::Ptr cornerCloud;
    pcl::PointCloud<PointType>::Ptr surfaceCloud;

    pcl::VoxelGrid<PointType> downSizeFilter;

    cloud_msg::msg::CloudInfo cloudInfo;
    std_msgs::msg::Header cloudHeader;

    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    int *cloudNeighborPicked;
    int *cloudLabel;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeatureExtraction>();
    node->initNode();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}