#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"
#include "imu_tracker.h"


Estimator estimator;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::msg::Imu::SharedPtr> imu_buf;
queue<sensor_msgs::msg::PointCloud::SharedPtr> feature_buf;

// global variable saving the lidar odometry
deque<nav_msgs::msg::Odometry> odomQueue;
odometryRegister *odomRegister;

std::mutex m_buf;
std::mutex m_state;
std::mutex m_estimator;
std::mutex m_odom;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;
std::shared_ptr<ImuTracker> imuTracker;

void predict(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    double t = imu_msg->header.stamp.sec+imu_msg->header.stamp.nanosec * (1e-9);
    if (init_imu)
    {
        latest_time = t;
        init_imu = false;

        imuTracker.reset(new ImuTracker(10.0, imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec*1e-9));

        return;
    }

//#define MODIFIED_PREINTEGRATION

#ifdef MODIFIED_PREINTEGRATION

    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    imuTracker->Advance(imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec*1e-9);
    imuTracker->AddImuLinearAccelerationObservation(linear_acceleration);
    imuTracker->AddImuAngularVelocityObservation(angular_velocity);

    tmp_Q = imuTracker->orientation();

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - imuTracker->gravity_vector_;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    //tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - imuTracker->gravity_vector_;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;

    // Debug
    Eigen::Vector3d rpy = tmp_Q.toRotationMatrix().eulerAngles(2, 1, 0)*180.0/M_PI;
    std::cout << "dt: " << dt << std::endl;
    std::cout << "dx, dy, dz: " << dx << " " << dy << " " << dz << std::endl;
    std::cout << std::endl
              << "Updated Position: " << tmp_P.transpose() << std::endl;
    std::cout << "Updated Velocity: " << tmp_V.transpose() << std::endl;
    std::cout << "Updated Orientation (RPY): Roll = "
              << rpy[2] << ", Pitch = " << rpy[1]
              << ", Yaw = " << rpy[0] << std::endl;
    std::cout << std::endl
              << "Updated Acceleration Bias: " << tmp_Ba.transpose() << std::endl;
    std::cout << "Updated Gyroscope Bias: " << tmp_Bg.transpose() << std::endl;
    std::cout << std::endl
              << "Estimator Gravity Vector: " << imuTracker->gravity_vector_.transpose() << std::endl;

#else // MODIFIED_PREINTEGRATION

    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;

    // Debug
     // Eigen::Vector3d rpy = tmp_Q.toRotationMatrix().eulerAngles(2, 1, 0)*180.0/M_PI;
     //       std::cout << "dt: " << dt << std::endl;
     // std::cout << "dx, dy, dz: " << dx << " " << dy << " " << dz << std::endl;
     // std::cout << std::endl
     //           << "Updated Position: " << tmp_P.transpose() << std::endl;
     // std::cout << "Updated Velocity: " << tmp_V.transpose() << std::endl;
     // std::cout << "Updated Orientation (RPY): Roll = "
     //           << rpy[2] << ", Pitch = " << rpy[1]
     //           << ", Yaw = " << rpy[0] << std::endl;
     // std::cout << std::endl
     //           << "Updated Acceleration Bias: " << tmp_Ba.transpose() << std::endl;
     // std::cout << "Updated Gyroscope Bias: " << tmp_Bg.transpose() << std::endl;
     // if (estimator.g.norm()>0.01)
     //     std::cout << std::endl
     //              << "Estimator Gravity Vector: " << estimator.g.transpose() << std::endl;
#endif
}

void update()
{
    cout<<"updating prediction";
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::msg::Imu::SharedPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::msg::Imu::SharedPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}

std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::SharedPtr>, sensor_msgs::msg::PointCloud::SharedPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::SharedPtr>, sensor_msgs::msg::PointCloud::SharedPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        if (!((imu_buf.back()->header.stamp.sec+imu_buf.back()->header.stamp.nanosec * (1e-9)) > (feature_buf.front()->header.stamp.sec+feature_buf.front()->header.stamp.nanosec * (1e-9)) + estimator.td))
        {
            //RCUTILS_LOG_WARN("wait for imu, only should happen at the beginning");
            return measurements;
        }

        if (!((imu_buf.front()->header.stamp.sec+imu_buf.front()->header.stamp.nanosec * (1e-9)) < (feature_buf.front()->header.stamp.sec+feature_buf.front()->header.stamp.nanosec * (1e-9)) + estimator.td))
        {
            RCUTILS_LOG_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::msg::PointCloud::SharedPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::msg::Imu::SharedPtr> IMUs;
        while ((imu_buf.front()->header.stamp.sec+imu_buf.front()->header.stamp.nanosec * (1e-9)) < (img_msg->header.stamp.sec+img_msg->header.stamp.nanosec * (1e-9)) + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            RCUTILS_LOG_WARN("no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    if ((imu_msg->header.stamp.sec+imu_msg->header.stamp.nanosec * (1e-9)) <= last_imu_t)
    {
        RCUTILS_LOG_WARN("imu message in disorder!");
        return;
    }

    imu_msg->linear_acceleration.x *= IMU_G;
    imu_msg->linear_acceleration.y *= IMU_G;
    imu_msg->linear_acceleration.z *= IMU_G;

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();


    last_imu_t = imu_msg->header.stamp.sec+imu_msg->header.stamp.nanosec * (1e-9);

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::msg::Header header = imu_msg->header;
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header, estimator.failureCount);
    }
}

void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
    m_odom.lock();
    odomQueue.push_back(*odom_msg);
    m_odom.unlock();
}

void feature_callback(const sensor_msgs::msg::PointCloud::SharedPtr feature_msg)
{
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

void restart_callback(const std_msgs::msg::Bool::SharedPtr restart_msg)
{
    if (restart_msg->data == true)
    {
        RCUTILS_LOG_WARN("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

// thread: visual-inertial odometry
void process()
{
    while (true)
    {
        std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::SharedPtr>, sensor_msgs::msg::PointCloud::SharedPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });

        lk.unlock();

        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;

            // 1. IMU pre-integration
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)
            {
                double t = imu_msg->header.stamp.sec+imu_msg->header.stamp.nanosec * (1e-9);
                double img_t = img_msg->header.stamp.sec+img_msg->header.stamp.nanosec * (1e-9) + estimator.td;
                if (t <= img_t)
                { 
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    assert(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);

                }
                else
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    assert(dt_1 >= 0);
                    assert(dt_2 >= 0);
                    assert(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }

            // 2. VINS Optimization

            TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                double depth = img_msg->channels[5].values[i];

                assert(z == 1);
                Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
                xyz_uv_velocity_depth << x, y, z, p_u, p_v, velocity_x, velocity_y, depth;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);
            }
            // Get initialization info from lidar odometry
            vector<float> initialization_info;
            m_odom.lock();
            initialization_info = odomRegister->getOdometry(odomQueue, img_msg->header.stamp.sec + img_msg->header.stamp.nanosec*1e-9 + estimator.td);
            m_odom.unlock();

            estimator.processImage(image, initialization_info, img_msg->header);

            std_msgs::msg::Header header = img_msg->header;

            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);

        }
        m_estimator.unlock();

        m_buf.lock();
        m_state.lock();
        cout<<"Got lock\n";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

int main(int argc, char **argv)
{
    // --ros-args -r __node:=estimator_node -p config_file:="/home/user/new_ws/install/config_pkg/share/config_pkg/config/garden/params_camera.yaml" -p vins_folder:="/home/user/new_ws/install/config_pkg/share/config_pkg/config/../"

    rclcpp::init(argc, argv);
    auto n = rclcpp::Node::make_shared("vins_estimator");
    readParameters(n);
    estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
    RCUTILS_LOG_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    RCUTILS_LOG_WARN("waiting for image and imu...");

    registerPub(n);

    odomRegister = new odometryRegister(n);

    auto sub_imu = n->create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, rclcpp::QoS(rclcpp::KeepLast(5000)).best_effort(), imu_callback);
    auto sub_odom = n->create_subscription<nav_msgs::msg::Odometry>("odometry/imu",rclcpp::QoS(rclcpp::KeepLast(5000)).best_effort(), odom_callback);
    auto sub_image = n->create_subscription<sensor_msgs::msg::PointCloud>("/vins/feature/feature", rclcpp::QoS(rclcpp::KeepLast(1)), feature_callback);
    auto sub_restart = n->create_subscription<std_msgs::msg::Bool>("/vins/feature/restart", rclcpp::QoS(rclcpp::KeepLast(1)), restart_callback);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> VINS Estimator Started.\033[0m");

    std::thread measurement_process{process};
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(n);
    executor.spin();

    return 0;
}
