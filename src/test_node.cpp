#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Include GTSAM
#include <gtsam/geometry/Point3.h>
#include <gtsam/slam/dataset.h>

// Include Ceres
#include <ceres/ceres.h>

// Include OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

// Sposta la struttura fuori dalla classe
struct QuadraticCost {
    template <typename T>
    bool operator()(const T* const x, T* residual) const
    {
        residual[0] = T(10.0) - x[0];
        return true;
    }
};

class DependencyTestNode : public rclcpp::Node
{
public:
    DependencyTestNode() : Node("dependency_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "Testing GTSAM...");
        testGTSAM();

        RCLCPP_INFO(this->get_logger(), "Testing Ceres...");
        testCeres();

        RCLCPP_INFO(this->get_logger(), "Testing OpenCV...");
        testOpenCV();
    }

private:
    void testGTSAM()
    {
        gtsam::Point3 point(1.0, 2.0, 3.0);
        RCLCPP_INFO(this->get_logger(), "GTSAM Point3: x=%f, y=%f, z=%f", point.x(), point.y(), point.z());
    }

    void testCeres()
    {
        double x = 0.0;
        ceres::Problem problem;
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<QuadraticCost, 1, 1>(new QuadraticCost),
            nullptr,
            &x);

        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = true;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        RCLCPP_INFO(this->get_logger(), "Ceres solved x: %f", x);
    }

    void testOpenCV()
    {
        cv::Mat image = cv::Mat::zeros(100, 100, CV_8UC3);
        cv::circle(image, cv::Point(50, 50), 25, cv::Scalar(0, 255, 0), -1);
        cv::imwrite("test_image.png", image);

        RCLCPP_INFO(this->get_logger(), "OpenCV: Created and saved an image with a circle.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DependencyTestNode>());
    rclcpp::shutdown();
    return 0;
}
