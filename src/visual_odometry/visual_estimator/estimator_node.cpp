#include <memory>
#include "parameters.h"
#include "estimator.h"
#include "utility/visualization.h"

class EstimatorNode : public rclcpp::Node
{
public:
    EstimatorNode() : Node("visual_estimator_node")
    {


        this->declare_parameter<std::string>("PROJECT_NAME", "emv_lio2");
        this->get_parameter("PROJECT_NAME", PROJECT_NAME);

        auto extRotV_lidar2imu = this->declare_parameter<std::vector<double>>("extrinsicRot", {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0});
        auto extTransV_lidar2imu = this->declare_parameter<std::vector<double>>("extrinsicTrans", {0.0,0.0,0.0});
        extRot_lidar2imu = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV_lidar2imu.data(), 3, 3);
        extTrans_lidar2imu = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV_lidar2imu.data(), 3, 1);

        NUM_OF_CAM = this->declare_parameter<int>("NUM_OF_CAM", 2);

        auto config_path = this->declare_parameter<std::string>("config_dir", "/home/user/ros2_ws/install/emv_lio2/share/emv_lio2/config");
        // std::cout<<config_path;
        readParameters(config_path);

        estimator.setParameter();

        //odomRegister = new odometryRegister(); // TODO: check, I removed the node handler from the class

        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, 5000,
            //std::bind(&EstimatorNode::imu_callback,this,std::placeholders::_1));
            [this](sensor_msgs::msg::Imu::SharedPtr msg){imu_callback(msg);});

        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("odometry/imu", 5000,
            std::bind(&EstimatorNode::odom_callback,this,std::placeholders::_1));

        sub_img0 = this->create_subscription<sensor_msgs::msg::PointCloud>(PROJECT_NAME + "/vins/feature/feature", 1,
            //std::bind(&EstimatorNode::feature_callback, this, std::placeholders::_1));
            [this](sensor_msgs::msg::PointCloud::SharedPtr msg){feature_callback(msg);});

        sub_img1 = this->create_subscription<sensor_msgs::msg::PointCloud>(PROJECT_NAME + "/vins/feature/feature_1", 1,
            //std::bind(&EstimatorNode::feature_callback1, this, std::placeholders::_1));
            [this](sensor_msgs::msg::PointCloud::SharedPtr msg){feature_callback1(msg);});

        sub_restart = this->create_subscription<std_msgs::msg::Bool>(PROJECT_NAME + "/vins/feature/restart", 1,
            //std::bind(&EstimatorNode::restart_callback, this, std::placeholders::_1));
            [this](std_msgs::msg::Bool::SharedPtr msg){restart_callback(msg);});

        RCLCPP_INFO(this->get_logger(), "\033[1;32mVisual Estimator Started.\033[0m");
    }

    void initPublishers()
    {
        registerPub(shared_from_this());
        odomRegister = new odometryRegister(shared_from_this());
        RCLCPP_INFO(this->get_logger(), "-----estimator's publishers registred-----");
    }

    void process()
    {
        while(rclcpp::ok())
        {
            std::vector<std::tuple<std::vector<sensor_msgs::msg::Imu::SharedPtr>,
                                   sensor_msgs::msg::PointCloud::SharedPtr,
                                   sensor_msgs::msg::PointCloud::SharedPtr>> measurements;
            std::unique_lock<std::mutex> lk(m_buf);
            con.wait(lk, [&]
                    {
                measurements = getMeasurements_two();
                auto measurements_size = measurements.size();
                // RCLCPP_INFO(this->get_logger(), "measurements size %lu ", measurements_size);
                return measurements_size != 0;
                    });
            lk.unlock();
            m_estimator.lock();
            RCLCPP_INFO(this->get_logger(), "PROCESSING MEASUREMENTS");
            for (auto &measurement : measurements)
            {
                auto img_msg = std::get<1>(measurement);
                auto img1_msg = std::get<2>(measurement);

                // 1. IMU pre-integration
                double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
                for (auto &imu_msg : std::get<0>(measurement))
                {
                    double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
                    double img_t = img_msg->header.stamp.sec + img_msg->header.stamp.nanosec * 1e-9 + estimator.td;
                    if (t <= img_t)
                    {
                        if (current_time < 0)
                            current_time = t;
                        double dt = t - current_time;
                        rcpputils::assert_true(dt >= 0, "EstimatorNode::process() assertion failed 1");
                        current_time = t;
                        dx = imu_msg->linear_acceleration.x;
                        dy = imu_msg->linear_acceleration.y;
                        dz = imu_msg->linear_acceleration.z;
                        rx = imu_msg->angular_velocity.x;
                        ry = imu_msg->angular_velocity.y;
                        rz = imu_msg->angular_velocity.z;
                        estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                        // printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                    }
                    else
                    {
                        double dt_1 = img_t - current_time;
                        double dt_2 = t - img_t;
                        current_time = img_t;
                        rcpputils::assert_true(dt_1 >= 0, "EstimatorNode::process() assertion failed 2");
                        rcpputils::assert_true(dt_2 >= 0, "EstimatorNode::process() assertion failed 3");
                        rcpputils::assert_true(dt_1 + dt_2 > 0, "EstimatorNode::process() assertion failed 3");
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
                // TicToc t_s;
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

                    rcpputils::assert_true(z == 1, "EstimatorNode::process() assertion failed 4");
                    Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
                    xyz_uv_velocity_depth << x, y, z, p_u, p_v, velocity_x, velocity_y, depth;
                    image[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);
                }
                map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> image1;
                for (unsigned int i = 0; i < img1_msg->points.size(); i++)
                {
                    int v1 = img1_msg->channels[0].values[i] + 0.5;
                    int feature1_id = v1 / NUM_OF_CAM;
                    int camera1_id = v1 % NUM_OF_CAM;
                    double x1 = img1_msg->points[i].x;
                    double y1 = img1_msg->points[i].y;
                    double z1 = img1_msg->points[i].z;
                    double p_u1 = img1_msg->channels[1].values[i];
                    double p_v1 = img1_msg->channels[2].values[i];
                    double velocity_x1 = img1_msg->channels[3].values[i];
                    double velocity_y1 = img1_msg->channels[4].values[i];
                    double depth1 = img1_msg->channels[5].values[i];

                    rcpputils::assert_true(z1 == 1, "EstimatorNode::process() assertion failed 5");
                    Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth1;
                    xyz_uv_velocity_depth1 << x1, y1, z1, p_u1, p_v1, velocity_x1, velocity_y1, depth1;
                    image1[feature1_id].emplace_back(camera1_id,  xyz_uv_velocity_depth1);
                }

                // Get initialization info from lidar odometry
                vector<float> initialization_info, initialization_info_1;
                m_odom.lock();
                initialization_info = odomRegister->getOdometry(odomQueue, img_msg->header.stamp.sec + img_msg->header.stamp.nanosec * 1e-9 + estimator.td, 0);
                initialization_info_1 = odomRegister->getOdometry(odomQueue, img1_msg->header.stamp.sec + img1_msg->header.stamp.nanosec * 1e-9 + estimator.td, 1);

                m_odom.unlock();
                estimator.processImage2(image, image1, initialization_info, initialization_info_1, img_msg->header);

                // double whole_t = t_s.toc();
                // printStatistics(estimator, whole_t);

                // 3. Visualization
                std_msgs::msg::Header header = img_msg->header;
                pubOdometry(estimator, header);
                pubKeyPoses(estimator, header);

                pubCameraPose(estimator, header);
                pubCameraPose1(estimator, header);

                pubPointCloud(estimator, header);
                pubPointCloud1(estimator, header);

                pubTF(estimator, header);
                pubKeyframe(estimator);

            }


            m_estimator.unlock();
            RCLCPP_INFO(this->get_logger(), "MEASUREMENTS PROCESSED");

            m_buf.lock();
            m_state.lock();
            if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
                update();
            m_state.unlock();
            m_buf.unlock();
        }
        RCLCPP_INFO(this->get_logger(), "terminating visual estimator process thread");

    }


private:
    void feature_callback(const sensor_msgs::msg::PointCloud::SharedPtr &feature_msg)
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
        //RCLCPP_INFO(this->get_logger(), "feature 0 callback: %lu features", feature_msg->points.size());
    }

    void feature_callback1(const sensor_msgs::msg::PointCloud::SharedPtr &feature_msg)
    {
        if (!init_feature_1)
        {
            //skip the first detected feature, which doesn't contain optical flow speed
            init_feature_1 = 1;
            return;
        }
        m_buf.lock();
        feature1_buf.push(feature_msg);
        m_buf.unlock();
        con.notify_one();
        //RCLCPP_INFO(this->get_logger(), "feature 1 callback: %lu features", feature_msg->points.size());
    }

    void restart_callback(const std_msgs::msg::Bool::SharedPtr& restart_msg)
    {
        if (restart_msg->data == true)
        {
            RCLCPP_WARN(this->get_logger(), "restart the estimator!");
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
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr& imu_msg)
    {
        if (imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9 <= last_imu_t)
        {
            RCLCPP_WARN(this->get_logger(), "imu message in disorder!");
            return;
        }
        last_imu_t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;

        m_buf.lock();
        imu_buf.push(imu_msg);
        m_buf.unlock();
        con.notify_one();

        // RCLCPP_INFO(this->get_logger(), "last_imu_t %f", last_imu_t);
        {
            std::lock_guard<std::mutex> lg(m_state);
            predict(imu_msg);

            std_msgs::msg::Header header = imu_msg->header;
            if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
                pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header, estimator.failureCount);
        }
        // RCLCPP_INFO(this->get_logger(), "imu callback works");
    }

    void odom_callback(const nav_msgs::msg::Odometry& odom_msg)
    {
        m_odom.lock();
        odomQueue.push_back(odom_msg);
        m_odom.unlock();
        RCLCPP_INFO(this->get_logger(), "odom callback works");
    }

    void predict(const sensor_msgs::msg::Imu::SharedPtr &imu_msg)
    {
        double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
        if (init_imu)
        {
            latest_time = t;
            init_imu = 0;
            return;
        }
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
    }

    void update()
    {
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

    std::vector<std::tuple<std::vector<sensor_msgs::msg::Imu::SharedPtr>,
                                   sensor_msgs::msg::PointCloud::SharedPtr,
                                   sensor_msgs::msg::PointCloud::SharedPtr>> getMeasurements_two()
    {
        std::vector<std::tuple<std::vector<sensor_msgs::msg::Imu::SharedPtr>,
                           sensor_msgs::msg::PointCloud::SharedPtr,
                           sensor_msgs::msg::PointCloud::SharedPtr>> measurements;
        while (rclcpp::ok())
        {
            if (imu_buf.empty() || feature_buf.empty() || feature1_buf.empty())
            {
                //RCLCPP_INFO(this->get_logger(), "empty buffers: %lu, %lu, %lu", imu_buf.size(), feature_buf.size(), feature1_buf.size());
                return measurements;
            }
            if (!(imu_buf.back()->header.stamp.sec + imu_buf.back()->header.stamp.nanosec * 1e-9 > feature_buf.front()->header.stamp.sec + feature_buf.front()->header.stamp.nanosec * 1e-9 + estimator.td ||
                    imu_buf.back()->header.stamp.sec + imu_buf.back()->header.stamp.nanosec * 1e-9 > feature1_buf.front()->header.stamp.sec + feature1_buf.front()->header.stamp.nanosec * 1e-9  + estimator.td))
            {
                return measurements;
            }

            if (!(imu_buf.front()->header.stamp.sec + imu_buf.front()->header.stamp.nanosec * 1e-9
                < feature_buf.front()->header.stamp.sec + feature_buf.front()->header.stamp.nanosec * 1e-9 + estimator.td))
            {
                RCLCPP_WARN(this->get_logger(), "throw img, only should happen at the beginning");
                feature_buf.pop();
                feature1_buf.pop();
                continue;
            }
            sensor_msgs::msg::PointCloud::SharedPtr img_msg = feature_buf.front();
            sensor_msgs::msg::PointCloud::SharedPtr img1_msg = feature1_buf.front();
            feature_buf.pop();
            feature1_buf.pop();

            std::vector<sensor_msgs::msg::Imu::SharedPtr> IMUs;
            while (imu_buf.front()->header.stamp.sec + imu_buf.front()->header.stamp.nanosec * 1e-9 < img_msg->header.stamp.sec + img_msg->header.stamp.nanosec * 1e-9 + estimator.td)
            {
                IMUs.emplace_back(imu_buf.front());
                imu_buf.pop();
            }
            IMUs.emplace_back(imu_buf.front());
            if (IMUs.empty())
                RCLCPP_WARN(this->get_logger(), "no imu between two image");
            measurements.emplace_back(IMUs, img_msg, img1_msg);
        }

        return measurements;
    }

    double latest_time;
    Eigen::Vector3d tmp_P;
    Eigen::Quaterniond tmp_Q;
    Eigen::Vector3d tmp_V;
    Eigen::Vector3d tmp_Ba;
    Eigen::Vector3d tmp_Bg;
    Eigen::Vector3d acc_0;
    Eigen::Vector3d gyr_0;
    bool init_feature = 0;
    bool init_feature_1 = 0;
    bool init_feature_2 = 0;
    bool init_imu = 1;
    double last_imu_t = 0;

    std::condition_variable con;
    double current_time = -1;
    queue<sensor_msgs::msg::Imu::SharedPtr> imu_buf;
    queue<sensor_msgs::msg::PointCloud::SharedPtr> feature_buf;
    queue<sensor_msgs::msg::PointCloud::SharedPtr> feature1_buf;
    //queue<sensor_msgs::msg::PointCloud::SharedPtr> feature2_buf;

    std::mutex m_buf;
    std::mutex m_state;
    std::mutex m_estimator;
    std::mutex m_odom;

    Estimator estimator;
    odometryRegister *odomRegister;
    deque<nav_msgs::msg::Odometry> odomQueue;


    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_img0;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_img1;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_restart;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EstimatorNode>();
    node->initPublishers();

    //node->startMeasurementThread();

    std::thread measurement_process(&EstimatorNode::process, node);
    RCLCPP_INFO(node->get_logger(), "measurement thread started");

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
    exec.add_node(node);
    RCLCPP_INFO(node->get_logger(), "NUM CAM %d",NUM_OF_CAM);
    exec.spin();
    rclcpp::shutdown();
    measurement_process.join();
    return 0;
}
