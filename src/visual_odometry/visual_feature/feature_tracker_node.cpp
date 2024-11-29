#include "feature_tracker.h"
#include "rclcpp/rclcpp.hpp"

std::mutex mtx_lidar;
pcl::PointCloud<PointType>::Ptr depthCloud(new pcl::PointCloud<PointType>());
deque<pcl::PointCloud<PointType>> cloudQueue;
deque<double> timeQueue;
DepthRegister *depthRegister;

class FeatureTrackerNode : public rclcpp::Node
{
public:
    FeatureTrackerNode() : Node("feature_tracker_node")
    {
        RCLCPP_INFO(this->get_logger(), "\033[1;32mVisual Feature Tracker Started.\033[0m");

    }

};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeatureTrackerNode>();
    readParameters(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}