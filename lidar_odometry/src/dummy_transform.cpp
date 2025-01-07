#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class StaticTransformPublisher : public rclcpp::Node
{
public:
    StaticTransformPublisher() : Node("static_transform_publisher")
    {
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Create and publish the identity transform
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "vins_world";
        transformStamped.child_frame_id = "vins_body_ros";
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;

        for(int i = 0; i<10000; i++)
        {
            static_broadcaster_->sendTransform(transformStamped);
            RCLCPP_INFO(this->get_logger(), "Published static identity transform.");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if(!rclcpp::ok()) break;
        }

    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticTransformPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
