#include "visualization.h"

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry, pub_latest_odometry, pub_latest_odometry_ros;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_point_cloud, pub_margin_cloud;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_point_cloud1, pub_margin_cloud1;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_point_cloud2, pub_margin_cloud2;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_key_poses;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_camera_pose, pub_camera_pose1, pub_camera_pose2;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_camera_pose_visual, pub_camera_pose_visual1, pub_camera_pose_visual2;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_keyframe_pose, pub_extrinsic;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_keyframe_point;

nav_msgs::msg::Path path;


CameraPoseVisualization cameraposevisual(0, 1, 0, 1);
CameraPoseVisualization cameraposevisual_1(0, 1, 0, 1);
CameraPoseVisualization cameraposevisual_2(0, 1, 0, 1);
CameraPoseVisualization keyframebasevisual(0.0, 0.0, 1.0, 1.0);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);
std::shared_ptr<rclcpp::Node> node;

void registerPub(std::shared_ptr<rclcpp::Node> node_in)
{
    node = node_in;

    pub_latest_odometry = node_in->create_publisher<nav_msgs::msg::Odometry>       ("vins/odometry/imu_propagate", 10);
    pub_latest_odometry_ros = node_in->create_publisher<nav_msgs::msg::Odometry>   ("vins/odometry/imu_propagate_ros", 10);
    pub_path = node_in->create_publisher<nav_msgs::msg::Path>                      ("vins/odometry/path", 10);
    pub_odometry = node_in->create_publisher<nav_msgs::msg::Odometry>              ("vins/odometry/odometry", 10);
    pub_point_cloud = node_in->create_publisher<sensor_msgs::msg::PointCloud>      ("vins/odometry/point_cloud", 10);
    pub_margin_cloud = node_in->create_publisher<sensor_msgs::msg::PointCloud>     ("vins/odometry/history_cloud", 10);
    pub_point_cloud1 = node_in->create_publisher<sensor_msgs::msg::PointCloud>     ("vins/odometry/point_cloud_1", 10);
    pub_margin_cloud1 = node_in->create_publisher<sensor_msgs::msg::PointCloud>    ("vins/odometry/history_cloud_1", 10);
    pub_point_cloud2 = node_in->create_publisher<sensor_msgs::msg::PointCloud>     ("vins/odometry/point_cloud_2", 10);
    pub_margin_cloud2 = node_in->create_publisher<sensor_msgs::msg::PointCloud>    ("vins/odometry/history_cloud_2", 10);
    pub_key_poses = node_in->create_publisher<visualization_msgs::msg::Marker>     ("vins/odometry/key_poses", 10);
    pub_camera_pose = node_in->create_publisher<nav_msgs::msg::Odometry>           ("vins/odometry/camera_pose", 10);
    pub_camera_pose1 = node_in->create_publisher<nav_msgs::msg::Odometry>          ("vins/odometry/camera_pose_1", 10);
    pub_camera_pose2 = node_in->create_publisher<nav_msgs::msg::Odometry>          ("vins/odometry/camera_pose_2", 10);
    pub_camera_pose_visual = node_in->create_publisher<visualization_msgs::msg::MarkerArray>
                                                                                ("vins/odometry/camera_pose_visual", 10);
    pub_camera_pose_visual1 = node_in->create_publisher<visualization_msgs::msg::MarkerArray>
                                                                                ("vins/odometry/camera_pose_visual_1", 10);
    pub_camera_pose_visual2 = node_in->create_publisher<visualization_msgs::msg::MarkerArray>
                                                                                ("vins/odometry/camera_pose_visual_2", 10);
    pub_keyframe_pose = node_in->create_publisher<nav_msgs::msg::Odometry>         ("vins/odometry/keyframe_pose", 10);
    pub_keyframe_point = node_in->create_publisher<sensor_msgs::msg::PointCloud>   ("vins/odometry/keyframe_point", 10);
    pub_extrinsic = node_in->create_publisher<nav_msgs::msg::Odometry>             ("vins/odometry/extrinsic", 10);


    cameraposevisual.setScale(1);
    cameraposevisual.setLineWidth(0.05);
    cameraposevisual_1.setScale(1);
    cameraposevisual_1.setLineWidth(0.05);
    cameraposevisual_2.setScale(1);
    cameraposevisual_2.setLineWidth(0.05);
    keyframebasevisual.setScale(0.1);
    keyframebasevisual.setLineWidth(0.01);
}

tf2::Transform transformConversion(const geometry_msgs::msg::TransformStamped& t)
{
    tf2::Transform transform;
    tf2::fromMsg(t.transform, transform);   // TODO fromMsg() check if implemented!
    RCLCPP_INFO(node->get_logger(), "Check if values match: translation %f, %f, %f", t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
    RCLCPP_INFO(node->get_logger(), ".. and ........................... %f, %f, %f", transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());

    RCLCPP_INFO(node->get_logger(), "Check if values match: rotation %f, %f, %f, %f", t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
    RCLCPP_INFO(node->get_logger(), ".. and ........................ %f, %f, %f, %f", transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ(), transform.getRotation().getW());

    return transform;
}

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V,
    const std_msgs::msg::Header &header, const int& failureId)
{
    static tf2_ros::TransformBroadcaster br(node);
    static tf2_ros::Buffer tfBuffer(node->get_clock());
    static tf2_ros::TransformListener listener(tfBuffer);
    static double last_align_time = -1;
    RCLCPP_INFO(node->get_logger(), "publishing latest odometry");

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

    // using params.yaml by sbq
    // TODO: matrix to quaternion needs revising
    tf2::Quaternion q_lidar_to_imu;
    double t;
    t = sqrt(1 + extRot_lidar2imu(0,0) + extRot_lidar2imu(1,1) + extRot_lidar2imu(2,2)) / 2;
    if(t == 0)
    {
        t = sqrt(1 + extRot_lidar2imu(0,0) - extRot_lidar2imu(1,1) - extRot_lidar2imu(2,2));
        q_lidar_to_imu.setValue(t / 4,
                                (extRot_lidar2imu(0,2) + extRot_lidar2imu(2,0)) / t,
                                (extRot_lidar2imu(1,0) + extRot_lidar2imu(0,1)) / t,
                                (extRot_lidar2imu(2,1) - extRot_lidar2imu(1,2)) / t);
        q_lidar_to_imu.normalize();
    }
    else
    {
        q_lidar_to_imu.setValue((extRot_lidar2imu(2,1) - extRot_lidar2imu(1,2)) / (4*t),
                                (extRot_lidar2imu(0,2) - extRot_lidar2imu(2,0)) / (4*t),
                                (extRot_lidar2imu(1,0) - extRot_lidar2imu(0,1)) / (4*t),
                                t);
        q_lidar_to_imu.normalize();
    }


    tf2::Quaternion q_imu_odom(Q.x(), Q.y(), Q.z(), Q.w());
    q_imu_odom.normalize();
    // q_imu_odom -->  imu2vins_world
    // vins_body = imu_link
    tf2::Quaternion q_odom_ros = q_imu_odom * q_lidar_to_imu;

    odometry.pose.pose.orientation = tf2::toMsg(q_odom_ros);
    pub_latest_odometry_ros->publish(odometry);

    // TF of camera in vins_world in ROS format (change rotation), used for depth registration
    // tf2::Transform t_w_body = tf2::Transform(q_odom_ros, tf2::Vector3(P.x(), P.y(), P.z()));
    // tf::StampedTransform trans_world_vinsbody_ros = tf::StampedTransform(t_w_body, header.stamp, "vins_world", "vins_body_ros");
    // br.sendTransform(trans_world_vinsbody_ros);  // TODO: se funziona è per opera dello spirito santo

    geometry_msgs::msg::TransformStamped trans_world_vinsbody_ros;
    trans_world_vinsbody_ros.header.stamp = header.stamp; // Ensure header stamp is of type builtin_interfaces::msg::Time
    trans_world_vinsbody_ros.header.frame_id = "vins_world";
    trans_world_vinsbody_ros.child_frame_id = "vins_body_ros";
    trans_world_vinsbody_ros.transform.translation.x = P.x();
    trans_world_vinsbody_ros.transform.translation.y = P.y();
    trans_world_vinsbody_ros.transform.translation.z = P.z();
    trans_world_vinsbody_ros.transform.rotation = tf2::toMsg(q_odom_ros);

    br.sendTransform(trans_world_vinsbody_ros);
    RCLCPP_INFO(node->get_logger(), "sending vins_world - vins_body_ros");

    if (ALIGN_CAMERA_LIDAR_COORDINATE)
    {
        // static tf::Transform t_odom_world = tf::Transform(tf::createQuaternionFromRPY(0, 0, M_PI), tf::Vector3(0, 0, 0));
        tf2::Quaternion q_odom_world;
        q_odom_world.setRPY(0, 0, M_PI);
        static tf2::Transform t_odom_world(q_odom_world, tf2::Vector3(0, 0, 0));

        if (header.stamp.sec + header.stamp.nanosec * 1e-9 - last_align_time > 1.0)
        {
            try
            {
                geometry_msgs::msg::TransformStamped trans_odom_baselink;
                trans_odom_baselink = tfBuffer.lookupTransform("odom", "base_link", tf2::TimePointZero);    // latest available transform
                t_odom_world = transformConversion(trans_odom_baselink) * transformConversion(trans_world_vinsbody_ros).inverse();
                last_align_time = header.stamp.sec + header.stamp.nanosec * 1e-9;
            }
            catch (tf2::TransformException& ex){}
        }

        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = header.stamp;
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "vins_world";

        transform_stamped.transform = tf2::toMsg(t_odom_world);

        br.sendTransform(transform_stamped);
        RCLCPP_INFO(node->get_logger(), "sending odom - vins_world");

    }
    else
    {
        geometry_msgs::msg::TransformStamped transform_stamped;

        tf2::Quaternion q_static;
        q_static.setRPY(0, 0, M_PI);

        // Populate the transform
        transform_stamped.header.stamp = header.stamp; // Ensure this is a valid builtin_interfaces::msg::Time
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "vins_world";
        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation = tf2::toMsg(q_static);

        br.sendTransform(transform_stamped);

        RCLCPP_INFO(node->get_logger(), "sending odom - vins_world");


    }
}

void printStatistics(const Estimator &estimator, double t)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    printf("position: %f, %f, %f\r", estimator.Ps[WINDOW_SIZE].x(), estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
    // ROS_DEBUG_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());
    // ROS_DEBUG_STREAM("orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        //ROS_DEBUG("calibration result for camera %d", i);
        // ROS_DEBUG_STREAM("extirnsic tic: " << estimator.tic[i].transpose());
        // ROS_DEBUG_STREAM("extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());
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
    // ROS_DEBUG("vo solver costs: %f ms", t);
    // ROS_DEBUG("average of time %f ms", sum_of_time / sum_of_calculation);

    sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator.Ps[WINDOW_SIZE];
    // ROS_DEBUG("sum of path %f", sum_of_path);
    if (ESTIMATE_TD)
        RCLCPP_INFO(node->get_logger(), "td %f", estimator.td);
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

        // cout << std::fixed << header.stamp.toSec() << " " << odometry.pose.pose.position.x
        //                                            << " " << odometry.pose.pose.position.y
        //                                            << " " << odometry.pose.pose.position.z
        //                                            << " " << odometry.pose.pose.orientation.x
        //                                            << " " << odometry.pose.pose.orientation.y
        //                                            << " " << odometry.pose.pose.orientation.z
        //                                            << " " << odometry.pose.pose.orientation.w << endl;
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
    key_poses.lifetime = rclcpp::Duration(0, 0);    // infinite marker duration

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

void pubCameraPose1(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if (pub_camera_pose_visual1->get_subscription_count() == 0)
        return;

    int idx2 = WINDOW_SIZE - 1;

    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[1];
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[1]);

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

        pub_camera_pose1->publish(odometry);

        cameraposevisual_1.reset();
        cameraposevisual_1.add_pose(P, R);
        cameraposevisual_1.publish_by(pub_camera_pose_visual1, odometry.header);
    }
}

void pubCameraPose2(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if (pub_camera_pose_visual2->get_subscription_count() == 0)
        return;

    int idx2 = WINDOW_SIZE - 1;

    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[2];
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[2]);

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

        pub_camera_pose2->publish(odometry);

        cameraposevisual_2.reset();
        cameraposevisual_2.add_pose(P, R);
        cameraposevisual_2.publish_by(pub_camera_pose_visual2, odometry.header);
    }
}


void pubPointCloud(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if (pub_point_cloud->get_subscription_count() != 0)
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
            if (it_per_id.cam_id != 0)
                continue;
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

            if (it_per_id.cam_id != 0)
                continue;
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

void pubPointCloud1(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if (pub_point_cloud1->get_subscription_count() != 0)
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
            if (it_per_id.cam_id != 1)
                continue;
            if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;
            if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
                continue;

            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            // estimator.ric[0] -> the transformation from cam to imu
            // w_pts_i -> the location of cam's features in vins_world coordinate
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[1] * pts_i + estimator.tic[1]) + estimator.Ps[imu_i];
            // Vector3d w_pts_i = estimator.Rs[imu_i] * pts_i + estimator.Ps[imu_i];

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
        pub_point_cloud1->publish(point_cloud);
    }

    // pub margined potin
    if (pub_margin_cloud1->get_subscription_count() != 0)
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

            if (it_per_id.cam_id != 1)
                continue;
            if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;

            if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2
                && it_per_id.solve_flag == 1 )
            {
                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[1] * pts_i + estimator.tic[1]) + estimator.Ps[imu_i];

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
        pub_margin_cloud1->publish(margin_cloud);
    }
}

void pubPointCloud2(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if (pub_point_cloud2->get_subscription_count() != 0)
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
            if (it_per_id.cam_id != 2)
                continue;
            if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;
            if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
                continue;

            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            // estimator.ric[0] -> the transformation from cam to imu
            // w_pts_i -> the location of cam's features in vins_world coordinate
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[2] * pts_i + estimator.tic[2]) + estimator.Ps[imu_i];
            // Vector3d w_pts_i = estimator.Rs[imu_i] * pts_i + estimator.Ps[imu_i];

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
        pub_point_cloud2->publish(point_cloud);
    }

    // pub margined potin
    if (pub_margin_cloud2->get_subscription_count() != 0)
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

            if (it_per_id.cam_id != 2)
                continue;
            if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;

            if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2
                && it_per_id.solve_flag == 1 )
            {
                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[2] * pts_i + estimator.tic[2]) + estimator.Ps[imu_i];

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
        pub_margin_cloud2->publish(margin_cloud);
    }
}

void pubTF(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if( estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    static tf2_ros::TransformBroadcaster br(node);
    tf2::Transform transform;
    tf2::Transform transform1;
    tf2::Quaternion q;
    // body frame
    Vector3d correct_t;
    Quaterniond correct_q;
    correct_t = estimator.Ps[WINDOW_SIZE];
    correct_q = estimator.Rs[WINDOW_SIZE];

    transform.setOrigin(tf2::Vector3(correct_t(0), correct_t(1), correct_t(2)));
    q.setW(correct_q.w());
    q.setX(correct_q.x());
    q.setY(correct_q.y());
    q.setZ(correct_q.z());
    transform.setRotation(q);

    geometry_msgs::msg::TransformStamped tmp_transform_stamped;

    tmp_transform_stamped.header.stamp = header.stamp;
    tmp_transform_stamped.header.frame_id = "vins_world";
    tmp_transform_stamped.child_frame_id = "vins_body";

    tmp_transform_stamped.transform = tf2::toMsg(transform);

    br.sendTransform(tmp_transform_stamped);
    RCLCPP_INFO(node->get_logger(), "sending vins_world - vins_body");


    // camera frame
    transform.setOrigin(tf2::Vector3(estimator.tic[0].x(),
                                    estimator.tic[0].y(),
                                    estimator.tic[0].z()));
    q.setW(Quaterniond(estimator.ric[0]).w());
    q.setX(Quaterniond(estimator.ric[0]).x());
    q.setY(Quaterniond(estimator.ric[0]).y());
    q.setZ(Quaterniond(estimator.ric[0]).z());
    transform.setRotation(q);

    tmp_transform_stamped.header.stamp = header.stamp;
    tmp_transform_stamped.header.frame_id = "vins_body";
    tmp_transform_stamped.child_frame_id = "vins_camera";

    tmp_transform_stamped.transform = tf2::toMsg(transform);

    br.sendTransform(tmp_transform_stamped);
    RCLCPP_INFO(node->get_logger(), "sending vins_body - vins_camera");



    transform1.setOrigin(tf2::Vector3(estimator.tic[1].x(),
                                    estimator.tic[1].y(),
                                    estimator.tic[1].z()));
    q.setW(Quaterniond(estimator.ric[1]).w());
    q.setX(Quaterniond(estimator.ric[1]).x());
    q.setY(Quaterniond(estimator.ric[1]).y());
    q.setZ(Quaterniond(estimator.ric[1]).z());
    transform1.setRotation(q);

    tmp_transform_stamped.header.stamp = header.stamp;
    tmp_transform_stamped.header.frame_id = "vins_body";
    tmp_transform_stamped.child_frame_id = "vins_camera_1";

    tmp_transform_stamped.transform = tf2::toMsg(transform1);

    br.sendTransform(tmp_transform_stamped);
    RCLCPP_INFO(node->get_logger(), "sending vins_world - vins_camera_1");


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