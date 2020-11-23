//################################################
#include "ndt_localization/ndt_localization.h"


PointCloudT new_scan;
bool newscan_Flag = false;
bool reset_initial_pose=true;
int new_scan_count=0;
int countLoop = 0;
ros::Time current_time;
Eigen::Matrix4f delta_newScanPose = Eigen::Isometry3f::Identity().matrix();
Eigen::Matrix4f last_tf = Eigen::Isometry3f::Identity().matrix();
PointCloudT mapcloud;
using namespace std;


NDTLocalization::~NDTLocalization()
{

}

bool NDTLocalization::init()
{
    ROS_INFO("Start init NDTLocalization");
    ros::Duration(1.0).sleep();

    pose_init_ = false;
    pub_target_map = nh_.advertise<sensor_msgs::PointCloud2>("local_target_map", 10);

    //pthread_mutex_init(&mutex, NULL);
    pnh_.param<bool>("is_filter_ground", is_filter_ground, true);
    pnh_.param<double>("min_scan_range", param_min_scan_range, 1.0);
    pnh_.param<double>("max_scan_range", param_max_scan_range, 100.0);
    std::cout << "min_scan_range: " << param_min_scan_range << std::endl;
    std::cout << "max_scan_range: " << param_max_scan_range << std::endl;
    std::cout << std::endl;

    pnh_.param<double>("voxel_leaf_size", voxel_leaf_size, 1.0);

    pnh_.param<std::string>("map_frame", param_map_frame_, std::string("/map"));
    pnh_.param<std::string>("odom_frame", param_odom_frame_, std::string("/odom"));
    pnh_.param<std::string>("base_frame", param_base_frame_, std::string("/base_link"));
    pnh_.param<std::string>("laser_frame", param_laser_frame_, std::string("/velodyne"));

    pnh_.param<std::string>("map_topic", param_map_topic_, std::string("/map"));
    pnh_.param<std::string>("odom_topic", param_odom_topic_, std::string("/odomImu/odom_imu"));
    pnh_.param<std::string>("lidar_topic", param_lidar_topic_, std::string("/velodyne_points"));
    std::cout << "*****lidar topic is " << param_lidar_topic_ << std::endl;

    pnh_.param<double>("tf_timeout", param_tf_timeout_, 0.05);
    pnh_.param<bool>("use_odom", param_use_odom_, false);
    pnh_.param<double>("odom_timeout", param_odom_timeout_, 1);
    if (param_use_odom_) 
        ROS_WARN_STREAM("Use odom.");
    else 
        ROS_WARN_STREAM("Forbid odom");
    
    pnh_.param<double>("predict_error_thresh", param_predict_error_thresh_, 0.4);
    pnh_.param<double>("ndt_resolution", param_ndt_resolution_, 1.0);
    pnh_.param<int>("ndt_max_iterations", param_ndt_max_iterations_, 25);
    pnh_.param<double>("ndt_step_size", param_ndt_step_size_, 0.1);
    pnh_.param<double>("ndt_epsilon", param_ndt_epsilon_, 0.01);
    
    pnh_.param<bool>("debug", param_debug_, false);
    pnh_.param<bool>("if_init_pose_with_param", param_init_pose_with_param, false);

    // 更新局部target地图相关参数
    pnh_.param<bool>("use_local_target", use_local_target, false);
    if (use_local_target) 
        ROS_WARN_STREAM("Use local target map");
    else 
        ROS_WARN_STREAM("Use global target map");
    
    pnh_.param<double>("target_map_radius", target_map_radius, 0.0);
    pnh_.param<double>("length_update_target_map", lengh_update_target_map, 1.0);
    pnh_.param<std::string>("global_map_file", map_file, "Confirm Location of Global Map.");

    pnh_.param<double>("length_update_path", length_update_path, 0.2);
    debug_path.header.frame_id = "map";


    // set t_btol and tf_btol.inverse   (base_link -> laser_link)
    tf::StampedTransform transform;
    try 
    {
        ros::Time now = ros::Time::now();
        ROS_INFO("now: %f", now.toSec()); //base_link       velodyne
        tf_listener_.waitForTransform(param_base_frame_, param_laser_frame_, ros::Time(0), ros::Duration(param_tf_timeout_ * 10), ros::Duration(param_tf_timeout_ / 3));
        tf_listener_.lookupTransform(param_base_frame_, param_laser_frame_, ros::Time(0), transform);
    }
    catch (const tf::TransformException& ex) {
        ROS_ERROR("Error waiting for tf in init: %s", ex.what());
        return false;
    }
    Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX()); //轴角  以x轴为轴
    Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());
    tf_btol_ = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix( );    // Eigen::Matrix4f tf_btol_;  base_link  to  velodyne

    if (param_init_pose_with_param) 
        init_pose_with_param();
    else 
        sub_initial_pose_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, boost::bind(&NDTLocalization::initialPoseCB, this, _1));

    load_map(map_file);

    while (!pose_init_) 
    {
        //ROS_WARN("initial pose not set!!!!!!");
        ros::spinOnce();
        //ROS消息回调处理函数。它俩通常会出现在ROS的主循环中，程序需要不断调用ros::spin() 或 ros::spinOnce()，
        //两者区别在于前者调用后不会再返回，也就是你的主程序到这儿就不往下执行了，而后者在调用后还可以继续执行之后的程序。
    }
    
    pub_current_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/ndt/current_pose", 10);
    pub_path = nh_.advertise<nav_msgs::Path>("/debug/history_path", 10);

    // pub_localPC_handled = nh_.advertise<sensor_msgs::PointCloud2>("/velodyne_no_ground", 10);

    ROS_INFO("End init NDTLocalization");
    return true;
}

void NDTLocalization::initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
   if(!pose_init_)
   {
    cout << "received a pose, init!!!!" << endl;
    init_Pose(0,3) = msg->pose.pose.position.x;
    init_Pose(1,3) = msg->pose.pose.position.y;
    init_Pose(2,3) = 0;  
    Eigen::Quaternionf temp(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);//w+xi+yj+zk
    init_Pose.block(0,0,3,3) = temp.matrix();
    pose_init_ = true;
   }
   else
   {
    cout << "received a pose, reset!!!!" << endl;
    init_Pose(0,3) = msg->pose.pose.position.x;
    init_Pose(1,3) = msg->pose.pose.position.y;
    init_Pose(2,3) = 0;  
    Eigen::Quaternionf temp(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);//w+xi+yj+zk
    init_Pose.block(0,0,3,3) = temp.matrix();
    pose_init_ = true;
   }
   newScan_Pose = init_Pose;
}

void NDTLocalization::init_pose_with_param()
{
    ROS_INFO("Init pose with param");
    pnh_.param<double>("init_x", initial_pose_.x, 0.0);
    pnh_.param<double>("init_y", initial_pose_.y, 0.0);
    pnh_.param<double>("init_z", initial_pose_.z, 0.0);
    pnh_.param<double>("init_roll", initial_pose_.roll, 0.0);
    pnh_.param<double>("init_pitch", initial_pose_.pitch, 0.0);
    pnh_.param<double>("init_yaw", initial_pose_.yaw, 0.0);

    pre_pose_ =  current_pose_ = initial_pose_;
    pose_init_ = true;

    std::cout << "Initial pose with:" << std::endl;
    std::cout << "    init_x: " << initial_pose_.x << std::endl;
    std::cout << "    init_y: " << initial_pose_.y << std::endl;
    std::cout << "    init_z: " << initial_pose_.z << std::endl;
    std::cout << " init_roll: " << initial_pose_.roll << std::endl;
    std::cout << "init_pitch: " << initial_pose_.pitch << std::endl;
    std::cout << "  init_yaw: " << initial_pose_.yaw << std::endl;
    ROS_INFO("Current pose initialized.");
}

/**
 * @brief 1. caculate pdf(probability density function)(mean, covariance) for each voxel grid in model
 * 
 * @param msg better to be filtered map data.
 */
bool NDTLocalization::load_map(std::string map_file)
{
    pub_global_map = nh_.advertise<sensor_msgs::PointCloud2>("/globalmap", 10);
    sensor_msgs::PointCloud2 msg_globalmap;
    PointCloudT map_cloud;
    pcl::io::loadPCDFile(map_file, map_cloud);
    pcl::toROSMsg(map_cloud, msg_globalmap);
    msg_globalmap.header.stamp = ros::Time::now();
    msg_globalmap.header.frame_id = "map";
    pub_global_map.publish(msg_globalmap);
    std::cout << "###Success load map: " << map_file << std::endl;
    // set NDT target

    PointCloudT::Ptr output_cloud(new PointCloudT());
    ndt_.setTransformationEpsilon(param_ndt_epsilon_);
    ndt_.setStepSize(param_ndt_step_size_);
    ndt_.setResolution(param_ndt_resolution_);
    ndt_.setMaximumIterations(param_ndt_max_iterations_);
    ndt_.setOulierRatio(0.15);
    ndt_.setInputTarget(map_cloud.makeShared());
    ndt_.align(*output_cloud, Eigen::Matrix4f::Identity());
    ndt_.initCompute();
    
    map_init_ = true;
    ROS_INFO("&&&&&&&&&&&&Update model pc with %d points!!!!!!!", map_cloud.width);
    return true;
}


void NDTLocalization::update_target_map()  // >>>>>>>>>>更新target地图  只与局部区域做匹配
{
    target_map_ptr->points.clear();
    for (auto point : model_pc_.points) {
        double dist = std::sqrt(std::pow(point.x - current_pose_.x, 2) + std::pow(point.y - current_pose_.y, 2));
        if (dist <= target_map_radius) {
            target_map_ptr->points.push_back(point);
        }
    }

    // publish target_map
    sensor_msgs::PointCloud2::Ptr msg_target_map_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*target_map_ptr, *msg_target_map_ptr);
    msg_target_map_ptr->header.frame_id = "map";
    pub_target_map.publish(*msg_target_map_ptr);

    ROS_WARN("update local map with %d points", target_map_ptr->points.size());
}

void NDTLocalization::pub_debug_path( )
{
    geometry_msgs::PoseStamped p;
    p.pose.position.x = current_pose_.x;
    p.pose.position.y = current_pose_.y;
    p.pose.position.z = current_pose_.z;

    Eigen::AngleAxisd roll_angle(current_pose_.roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(current_pose_.pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(current_pose_.yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = roll_angle * pitch_angle * yaw_angle;
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();

    debug_path.poses.push_back(p);
    pub_path.publish(debug_path);
}


void NDTLocalization::pointCloud_process( )
{
    
    PointCloudT::Ptr scan_ptr(new PointCloudT());
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_grid_filter.setInputCloud(new_scan.makeShared( ) );
    voxel_grid_filter.filter(*scan_ptr);
    // ROS_WARN("filtered size: %d",scan_ptr->points.size());

    Eigen::Matrix4f init_guess;
    Eigen::Matrix4f final_tf;
    Eigen::Matrix4f base_tf;
    pose ndt_pose;

    PointCloudT::Ptr output_cloud(new PointCloudT());
    ndt_.setInputSource(scan_ptr);
    ndt_.align(*output_cloud, newScan_Pose);

    newScan_Pose = ndt_.getFinalTransformation();
    delta_newScanPose = last_tf.inverse() * newScan_Pose;
    // base_tf = final_tf * tf_btol_.inverse();
    base_tf = newScan_Pose;
    // TODO need to transform from base to lidar
    //  base_tf = transform * base_tf;
    tf::Matrix3x3 mat_b;  //旋转矩阵
    mat_b.setValue(static_cast<double>(base_tf(0, 0)), static_cast<double>(base_tf(0, 1)), static_cast<double>(base_tf(0, 2)),
                   static_cast<double>(base_tf(1, 0)), static_cast<double>(base_tf(1, 1)), static_cast<double>(base_tf(1, 2)),
                   static_cast<double>(base_tf(2, 0)), static_cast<double>(base_tf(2, 1)), static_cast<double>(base_tf(2, 2)));
    //平移
    ndt_pose.x = base_tf(0, 3);
    ndt_pose.y = base_tf(1, 3);
    ndt_pose.z = base_tf(2, 3);
    mat_b.getEulerYPR(ndt_pose.yaw, ndt_pose.pitch, ndt_pose.roll);

    pose2GeometryPose(msg_current_pose_.pose, current_pose_);
    msg_current_pose_.header.stamp = ros::Time::now(); // current pose is under "map_frame"
    msg_current_pose_.header.frame_id = "map";  //  map
    pub_current_pose_.publish(msg_current_pose_);

    current_pose_ = ndt_pose;  //全局位置current_pose
    
    tf::Quaternion tmp_q;
    tmp_q.setRPY(current_pose_.roll, current_pose_.pitch, current_pose_.yaw);
    tf::Transform map_to_laser(tmp_q, tf::Vector3(current_pose_.x, current_pose_.y, current_pose_.z + 0.35));
    // tf_broadcaster_.sendTransform(tf::StampedTransform(map_to_laser, ros::Time::now(), param_map_frame_, param_laser_frame_));

    tf::StampedTransform base_to_laser;
    try{
        base_laser_tf_listener_.lookupTransform(param_base_frame_, param_laser_frame_, ros::Time(0), base_to_laser);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    tf::Transform laser_to_base = base_to_laser.inverse();
    tf::Transform map_to_base;
    map_to_base.mult(map_to_laser, laser_to_base);
    tf_broadcaster_.sendTransform(tf::StampedTransform(map_to_base, ros::Time::now(), param_map_frame_, param_base_frame_));

    pre_pose_ = current_pose_;
}

void pointCloudCB(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    current_time = msg->header.stamp;
    pcl::fromROSMsg(*msg, new_scan);
    newscan_Flag = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ndt_localization_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
   
  NDTLocalization ndt(nh, pnh);
  ndt.init();  //初始化!!!!!!!!!&&&&&&&&&&&&&&&&&&&
  ros::Subscriber sub_point_cloud_ = nh.subscribe<sensor_msgs::PointCloud2>(ndt.param_lidar_topic_, 10, pointCloudCB,ros::TransportHints().tcpNoDelay());

  ros::Rate rate(100); //ros::Rate对象允许你制定循环的频率。它将会记录从上次调用Rate::sleep()到现在为止的时间，并且休眠正确的时间。在这个例子中，设置的频率为10hz。

  while(ros::ok())
  {
    if(newscan_Flag)
    {
        newscan_Flag = false;
        ndt.pointCloud_process( );
    }

    ros::spinOnce();//查询回调函数中断标志位。执行回调函数。
    rate.sleep();//以ros::Rate来延时
  }
  return 0;
}
