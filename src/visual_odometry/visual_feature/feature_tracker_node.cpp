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

        // Lidars parameters
        this->declare_parameter<std::string>("PROJECT_NAME", "emv-lio2");
        this->get_parameter("PROJECT_NAME", PROJECT_NAME);

        auto extRotV_lidar2imu = this->declare_parameter<std::vector<double>>(PROJECT_NAME + ".extrinsicRot", {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0});
        auto extTransV_lidar2imu = this->declare_parameter<std::vector<double>>(PROJECT_NAME + ".extrinsicTrans", {0.0,0.0,0.0});
        extRot_lidar2imu = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV_lidar2imu.data(), 3, 3);
        extTrans_lidar2imu = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV_lidar2imu.data(), 3, 1);

        NUM_OF_CAM = this->declare_parameter<int>(PROJECT_NAME + ".NUM_OF_CAM", 1);

        // Load config files
        // std::string pkg_path = ament_index_cpp::get_package_share_directory(PROJECT_NAME);
        auto config_path = this->declare_parameter<std::string>("config_dir", "/home/user/ros2_ws/install/emv-lio2/share/emv-lio2/config");
        readParameters(config_path);


    }

};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeatureTrackerNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}