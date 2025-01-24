#include "visualization.h"

using namespace Eigen;

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_latest_odometry;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_latest_odometry_ros;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
// rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_relo_path;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_point_cloud;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_margin_cloud;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_key_poses;
// rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_relo_relative_pose;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_camera_pose;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_camera_pose_visual;
nav_msgs::msg::Path path;

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_keyframe_pose;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_keyframe_point;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_extrinsic;

CameraPoseVisualization cameraposevisual(0, 1, 0, 1);
CameraPoseVisualization keyframebasevisual(0.0, 0.0, 1.0, 1.0);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);

static std::shared_ptr<tf2_ros::TransformBroadcaster> br;
static std::shared_ptr<tf2_ros::Buffer> tfBuffer;
static std::shared_ptr<tf2_ros::TransformListener> tfListener;

void registerPub(rclcpp::Node::SharedPtr n)
{
    pub_latest_odometry = n->create_publisher<nav_msgs::msg::Odometry>(         "/vins/odometry/imu_propagate", 1000);
    pub_latest_odometry_ros = n->create_publisher<nav_msgs::msg::Odometry>(     "/vins/odometry/imu_propagate_ros", 1000);
    pub_path = n->create_publisher<nav_msgs::msg::Path>(                        "/vins/odometry/path", 1000);
    // pub_relo_path = n->create_publisher<nav_msgs::msg::Path>("relocalization_path", 1000);
    pub_odometry = n->create_publisher<nav_msgs::msg::Odometry>(                "/vins/odometry/odometry", 1000);
    pub_point_cloud = n->create_publisher<sensor_msgs::msg::PointCloud>(        "/vins/odometry/point_cloud", 1000);
    pub_margin_cloud = n->create_publisher<sensor_msgs::msg::PointCloud>(       "/vins/odometry/history_cloud", 1000);
    pub_key_poses = n->create_publisher<visualization_msgs::msg::Marker>(       "/vins/odometry/key_poses", 1000);
    pub_camera_pose = n->create_publisher<nav_msgs::msg::Odometry>(             "/vins/odometry/camera_pose", 1000);
    pub_camera_pose_visual = n->create_publisher<visualization_msgs::msg::MarkerArray>("/vins/odometry/camera_pose_visual", 1000);
    pub_keyframe_pose = n->create_publisher<nav_msgs::msg::Odometry>(           "/vins/odometry/keyframe_pose", 1000);
    pub_keyframe_point = n->create_publisher<sensor_msgs::msg::PointCloud>(     "/vins/odometry/keyframe_point", 1000);
    pub_extrinsic = n->create_publisher<nav_msgs::msg::Odometry>(               "/vins/odometry/extrinsic", 1000);
    // pub_relo_relative_pose=  n->create_publisher<nav_msgs::msg::Odometry>("relo_relative_pose", 1000);

    br = std::make_shared<tf2_ros::TransformBroadcaster>(n);
    tfBuffer = std::make_shared<tf2_ros::Buffer>(n->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
    cameraposevisual.setScale(1);
    cameraposevisual.setLineWidth(0.05);
    keyframebasevisual.setScale(0.1);
    keyframebasevisual.setLineWidth(0.01);
}

tf2::Transform transformConversion(const geometry_msgs::msg::TransformStamped& t)
{
    double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
    xCur = t.transform.translation.x;
    yCur = t.transform.translation.y;
    zCur = t.transform.translation.z;

    tf2::Quaternion q;
    tf2::fromMsg(t.transform.rotation,q);

    return tf2::Transform(q, tf2::Vector3(xCur, yCur, zCur));;
}

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const std_msgs::msg::Header &header, const int& failureId)
{
    static double last_align_time = -1;

    // Quternion not normalized
    if (Q.x() * Q.x() + Q.y() * Q.y() + Q.z() * Q.z() + Q.w() * Q.w() < 0.99)
        return;

    // imu odometry in camera frame
    nav_msgs::msg::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "vins_world";
    odometry.child_frame_id = "vins_body";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();
    odometry.pose.pose.orientation.w = Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    pub_latest_odometry->publish(odometry);

    // imu odometry in ROS format (change rotation), used for lidar odometry initial guess
    odometry.pose.covariance[0] = double(failureId); // notify lidar odometry failure

    tf2::Quaternion q_odom_cam(Q.x(), Q.y(), Q.z(), Q.w());
    tf2::Quaternion q_cam_to_lidar(0, 1, 0, 0); // mark: camera - lidar
    tf2::Quaternion q_odom_ros = q_odom_cam * q_cam_to_lidar;

    odometry.pose.pose.orientation = tf2::toMsg(q_odom_ros);
    pub_latest_odometry_ros->publish(odometry);

    // TF of camera in vins_world in ROS format (change rotation), used for depth registration
    tf2::Transform t_w_body = tf2::Transform(q_odom_ros, tf2::Vector3(P.x(), P.y(), P.z()));
    // tf::StampedTransform trans_world_vinsbody_ros = tf::StampedTransform(t_w_body, header.stamp, "vins_world", "vins_body_ros");
    geometry_msgs::msg::TransformStamped trans_world_vinsbody_ros;
    trans_world_vinsbody_ros.header.frame_id = "vins_world";
    trans_world_vinsbody_ros.child_frame_id = "vins_body_ros";
    trans_world_vinsbody_ros.header.stamp = header.stamp;
    trans_world_vinsbody_ros.transform = tf2::toMsg(t_w_body);

    br->sendTransform(trans_world_vinsbody_ros);

    if (ALIGN_CAMERA_LIDAR_COORDINATE)
    {
        static tf2::Transform t_odom_world = []() {
            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            return tf2::Transform(q, tf2::Vector3(0, 0, 0));
        }();

        if (header.stamp.sec + header.stamp.nanosec*1e-9 - last_align_time > 1.0)
        {
            try
            {
                tf2::Stamped<tf2::Transform> trans_odom_baselink;
                tf2::fromMsg(tfBuffer->lookupTransform("odom", "base_link", rclcpp::Time(0)), trans_odom_baselink);
                t_odom_world = trans_odom_baselink * transformConversion(trans_world_vinsbody_ros).inverse();
                last_align_time = header.stamp.sec + header.stamp.nanosec*1e-9;
            }
            catch (tf2::TransformException& ex){}
        }
        geometry_msgs::msg::TransformStamped stamped_odom_world;

        stamped_odom_world.header.stamp = header.stamp;
        stamped_odom_world.header.frame_id = "odom";
        stamped_odom_world.child_frame_id = "vins_world";

        stamped_odom_world.transform.translation.x = t_odom_world.getOrigin().x();
        stamped_odom_world.transform.translation.y = t_odom_world.getOrigin().y();
        stamped_odom_world.transform.translation.z = t_odom_world.getOrigin().z();

        stamped_odom_world.transform.rotation.x = t_odom_world.getRotation().x();
        stamped_odom_world.transform.rotation.y = t_odom_world.getRotation().y();
        stamped_odom_world.transform.rotation.z = t_odom_world.getRotation().z();
        stamped_odom_world.transform.rotation.w = t_odom_world.getRotation().w();

        br->sendTransform(stamped_odom_world);
    }
    else
    {
        geometry_msgs::msg::TransformStamped static_transform_stamped;

        tf2::Transform t_static;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        t_static.setRotation(q);
        t_static.setOrigin(tf2::Vector3(0, 0, 0));

        static_transform_stamped.header.stamp = header.stamp;
        static_transform_stamped.header.frame_id = "odom";
        static_transform_stamped.child_frame_id = "vins_world";
        static_transform_stamped.transform.translation.x = t_static.getOrigin().x();
        static_transform_stamped.transform.translation.y = t_static.getOrigin().y();
        static_transform_stamped.transform.translation.z = t_static.getOrigin().z();
        static_transform_stamped.transform.rotation.x = t_static.getRotation().x();
        static_transform_stamped.transform.rotation.y = t_static.getRotation().y();
        static_transform_stamped.transform.rotation.z = t_static.getRotation().z();
        static_transform_stamped.transform.rotation.w = t_static.getRotation().w();

        br->sendTransform(static_transform_stamped);
    }
}

void printStatistics(const Estimator &estimator, double t)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    printf("position: %f, %f, %f\r", estimator.Ps[WINDOW_SIZE].x(), estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
    std::cout << "position: " << estimator.Ps[WINDOW_SIZE].transpose() << std::endl;
    std::cout << "orientation: " << estimator.Vs[WINDOW_SIZE].transpose() << std::endl;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        //ROS_DEBUG("calibration result for camera %d", i);
        std::cout << "extirnsic tic: " << estimator.tic[i].transpose() << std::endl;
        std::cout << "extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose() << std::endl;
        if (ESTIMATE_EXTRINSIC)
        {
            cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
            Eigen::Matrix3d eigen_R;
            Eigen::Vector3d eigen_T;
            eigen_R = estimator.ric[i];
            eigen_T = estimator.tic[i];
            cv::Mat cv_R, cv_T;
            cv::eigen2cv(eigen_R, cv_R);
            cv::eigen2cv(eigen_T, cv_T);
            fs << "extrinsicRotation" << cv_R << "extrinsicTranslation" << cv_T;
            fs.release();
        }
    }

    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    RCUTILS_LOG_DEBUG("vo solver costs: %f ms", t);
    RCUTILS_LOG_DEBUG("average of time %f ms", sum_of_time / sum_of_calculation);

    sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator.Ps[WINDOW_SIZE];
    RCUTILS_LOG_DEBUG("sum of path %f", sum_of_path);
    if (ESTIMATE_TD)
        RCUTILS_LOG_INFO("td %f", estimator.td);
}

void pubOdometry(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        nav_msgs::msg::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "vins_world";
        odometry.child_frame_id = "vins_world";
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
        odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
        odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
        odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
        pub_odometry->publish(odometry);

        static double path_save_time = -1;
        if (header.stamp.sec + header.stamp.nanosec * 1e-9 - path_save_time > 0.5)
        {
            path_save_time = header.stamp.sec + header.stamp.nanosec * 1e-9;
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = header;
            pose_stamped.header.frame_id = "vins_world";
            pose_stamped.pose = odometry.pose.pose;
            path.header = header;
            path.header.frame_id = "vins_world";
            path.poses.push_back(pose_stamped);
            pub_path->publish(path);
        }
    }
}

void pubKeyPoses(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if (pub_key_poses->get_subscription_count() == 0)
        return;

    if (estimator.key_poses.size() == 0)
        return;
    visualization_msgs::msg::Marker key_poses;
    key_poses.header = header;
    key_poses.header.frame_id = "vins_world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::msg::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = rclcpp::Duration(0, 0);

    //static int key_poses_id = 0;
    key_poses.id = 0; //key_poses_id++;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        geometry_msgs::msg::Point pose_marker;
        Vector3d correct_pose;
        correct_pose = estimator.key_poses[i];
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker);
    }
    pub_key_poses->publish(key_poses);
}

void pubCameraPose(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if (pub_camera_pose_visual->get_subscription_count() == 0)
        return;

    int idx2 = WINDOW_SIZE - 1;

    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);

        nav_msgs::msg::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "vins_world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_camera_pose->publish(odometry);

        cameraposevisual.reset();
        cameraposevisual.add_pose(P, R);
        cameraposevisual.publish_by(pub_camera_pose_visual, odometry.header);
    }
}


void pubPointCloud(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if(pub_point_cloud->get_subscription_count() != 0)
    {
        sensor_msgs::msg::PointCloud point_cloud;
        point_cloud.header = header;
        point_cloud.header.frame_id = "vins_world";

        sensor_msgs::msg::ChannelFloat32 intensity_channel;
        intensity_channel.name = "intensity";

        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int used_num;
            used_num = it_per_id.feature_per_frame.size();
            if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;
            if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
                continue;

            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

            geometry_msgs::msg::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            point_cloud.points.push_back(p);
            if (it_per_id.lidar_depth_flag == false)
                intensity_channel.values.push_back(0);
            else
                intensity_channel.values.push_back(1);
        }
        point_cloud.channels.push_back(intensity_channel);
        pub_point_cloud->publish(point_cloud);
    }

    // pub margined potin
    if (pub_margin_cloud->get_subscription_count() != 0)
    {
        sensor_msgs::msg::PointCloud margin_cloud;
        margin_cloud.header = header;
        margin_cloud.header.frame_id = "vins_world";

        sensor_msgs::msg::ChannelFloat32 intensity_channel;
        intensity_channel.name = "intensity";

        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int used_num;
            used_num = it_per_id.feature_per_frame.size();
            if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;

            if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2
                && it_per_id.solve_flag == 1 )
            {
                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

                geometry_msgs::msg::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                margin_cloud.points.push_back(p);

                if (it_per_id.lidar_depth_flag == false)
                    intensity_channel.values.push_back(0);
                else
                    intensity_channel.values.push_back(1);
            }
        }

        margin_cloud.channels.push_back(intensity_channel);
        pub_margin_cloud->publish(margin_cloud);
    }
}


void pubTF(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if( estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;

    geometry_msgs::msg::TransformStamped transform, transform_cam;

    // body frame
    Vector3d correct_t;
    Quaterniond correct_q;
    correct_t = estimator.Ps[WINDOW_SIZE];
    correct_q = estimator.Rs[WINDOW_SIZE];

    transform.header.stamp = header.stamp;
    transform.header.frame_id = "vins_world";
    transform.child_frame_id = "vins_body";

    transform.transform.translation.x = correct_t(0);
    transform.transform.translation.y = correct_t(1);
    transform.transform.translation.z = correct_t(2);


    transform.transform.rotation.x = correct_q.x();
    transform.transform.rotation.y = correct_q.y();
    transform.transform.rotation.z = correct_q.z();
    transform.transform.rotation.w = correct_q.w();

    br->sendTransform(transform);

    // camera frame
    transform_cam.header.stamp = header.stamp;
    transform_cam.header.frame_id = "vins_body";
    transform_cam.child_frame_id = "vins_camera";


    transform_cam.transform.translation.x = estimator.tic[0].x();
    transform_cam.transform.translation.y = estimator.tic[0].y();
    transform_cam.transform.translation.z = estimator.tic[0].z();

    tf2::Quaternion q;
    q.setW(Quaterniond(estimator.ric[0]).w());
    q.setX(Quaterniond(estimator.ric[0]).x());
    q.setY(Quaterniond(estimator.ric[0]).y());
    q.setZ(Quaterniond(estimator.ric[0]).z());

    transform_cam.transform.rotation.x = q.x();
    transform_cam.transform.rotation.y = q.y();
    transform_cam.transform.rotation.z = q.z();
    transform_cam.transform.rotation.w = q.w();

    br->sendTransform(transform_cam);

    nav_msgs::msg::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "vins_world";
    odometry.pose.pose.position.x = estimator.tic[0].x();
    odometry.pose.pose.position.y = estimator.tic[0].y();
    odometry.pose.pose.position.z = estimator.tic[0].z();
    Quaterniond tmp_q{estimator.ric[0]};
    odometry.pose.pose.orientation.x = tmp_q.x();
    odometry.pose.pose.orientation.y = tmp_q.y();
    odometry.pose.pose.orientation.z = tmp_q.z();
    odometry.pose.pose.orientation.w = tmp_q.w();
    pub_extrinsic->publish(odometry);

}

void pubKeyframe(const Estimator &estimator)
{
    if (pub_keyframe_pose->get_subscription_count() == 0 && pub_keyframe_point->get_subscription_count() == 0)
        return;

    // pub camera pose, 2D-3D points of keyframe
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR && estimator.marginalization_flag == 0)
    {
        int i = WINDOW_SIZE - 2;
        //Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Vector3d P = estimator.Ps[i];
        Quaterniond R = Quaterniond(estimator.Rs[i]);

        nav_msgs::msg::Odometry odometry;
        odometry.header = estimator.Headers[WINDOW_SIZE - 2];
        odometry.header.frame_id = "vins_world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();
        //printf("time: %f t: %f %f %f r: %f %f %f %f\n", odometry.header.stamp.toSec(), P.x(), P.y(), P.z(), R.w(), R.x(), R.y(), R.z());

        pub_keyframe_pose->publish(odometry);


        sensor_msgs::msg::PointCloud point_cloud;
        point_cloud.header = estimator.Headers[WINDOW_SIZE - 2];
        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int frame_size = it_per_id.feature_per_frame.size();
            if(it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 && it_per_id.solve_flag == 1)
            {

                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0])
                                      + estimator.Ps[imu_i];
                geometry_msgs::msg::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                point_cloud.points.push_back(p);

                int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;
                sensor_msgs::msg::ChannelFloat32 p_2d;
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.y());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.y());
                p_2d.values.push_back(it_per_id.feature_id);
                point_cloud.channels.push_back(p_2d);
            }

        }
        pub_keyframe_point->publish(point_cloud);
    }
}
