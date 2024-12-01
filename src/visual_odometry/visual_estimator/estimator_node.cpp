#include <memory>
#include "parameters.h"


class EstimatorNode : public rclcpp::Node
{
public:
    EstimatorNode() : Node("visual_estimator_node")
    {
        RCLCPP_INFO(this->get_logger(), "\033[1;32mVisual Feature Tracker Started.\033[0m");

        this->declare_parameter<std::string>("PROJECT_NAME", "emv_lio2");
        this->get_parameter("PROJECT_NAME", PROJECT_NAME);

        auto extRotV_lidar2imu = this->declare_parameter<std::vector<double>>("extrinsicRot", {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0});
        auto extTransV_lidar2imu = this->declare_parameter<std::vector<double>>("extrinsicTrans", {0.0,0.0,0.0});
        extRot_lidar2imu = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV_lidar2imu.data(), 3, 3);
        extTrans_lidar2imu = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV_lidar2imu.data(), 3, 1);

        NUM_OF_CAM = this->declare_parameter<int>("NUM_OF_CAM", 1);

        auto config_path = this->declare_parameter<std::string>("config_dir", "/home/user/ros2_ws/install/emv_lio2/share/emv_lio2/config");

        readParameters(config_path);

        // TODO estimator.setParameter();

        // TODO: registerPub(this);

        // TODO: odomRegister = new odometryRegister(this);

        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, 5000,
            std::bind(&EstimatorNode::imu_callback,this,std::placeholders::_1));
        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("odometry/imu", 5000,
            std::bind(&EstimatorNode::odom_callback,this,std::placeholders::_1));
        sub_img0 = this->create_subscription<sensor_msgs::msg::PointCloud>(PROJECT_NAME + "/vins/feature/feature", 1,
            std::bind(&EstimatorNode::feature_callback, this, std::placeholders::_1));
        sub_img1 = this->create_subscription<sensor_msgs::msg::PointCloud>(PROJECT_NAME + "/vins/feature/feature1", 1,
            std::bind(&EstimatorNode::feature_callback, this, std::placeholders::_1));
        sub_restart = this->create_subscription<std_msgs::msg::Bool>(PROJECT_NAME + "/vins/feature/restart", 1,
            std::bind(&EstimatorNode::restart_callback, this, std::placeholders::_1));


    }

private:
    void feature_callback(const sensor_msgs::msg::PointCloud &feature_msg){}
    void feature_callback1(const sensor_msgs::msg::PointCloud &feature_msg){}
    void restart_callback(const std_msgs::msg::Bool& restart_msg){}
    void imu_callback(const sensor_msgs::msg::Imu& imu_msg){}
    void odom_callback(const nav_msgs::msg::Odometry& odom_msg){}

    // Estimator estimator;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_img0;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_img1;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_restart;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EstimatorNode>();

    //std::thread measurement_process{process};

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    // measurement_process.join();
    return 0;
}
