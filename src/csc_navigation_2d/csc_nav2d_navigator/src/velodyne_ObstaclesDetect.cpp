#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <tic_toc.h>

using namespace std;

ros::Publisher obstacles_pub;
ros::Publisher marker_pub;

float detect_angle = M_PI_4;
float danger_range;
float region_radius;

void scan_callback(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
    //ROS_INFO_STREAM("velodyne callback!");

    int start_index = (-detect_angle + M_PI) / scan_msg->angle_increment;
    int stop_index = (detect_angle + M_PI) / scan_msg->angle_increment;

    std_msgs::Float32 range_data;

    range_data.data = INFINITY;

    // 检测扇形区域的障碍物
    /*for(int i=start_index; i <= stop_index; i++)
    {
        if(scan_msg->ranges[i] <= danger_range)
        {
            ROS_WARN_STREAM("Detect obstacles!!!");
            range_data.data = true;
            break;
        }
    }*/

    float obstacles_range = 0.0;
    // 检测方形区域的障碍物
    for(int i=0; i <= scan_msg->ranges.size(); i++)
    {
        float angle = i * scan_msg->angle_increment - M_PI;
        if(fabs(scan_msg->ranges[i] * sin(angle)) < danger_range && scan_msg->ranges[i] * cos(angle) > 0.0)
        {
            obstacles_range = scan_msg->ranges[i] * cos(angle);
            if(obstacles_range < range_data.data)
                range_data.data = obstacles_range;
        }
    }

    obstacles_pub.publish(range_data);
}

vector<tf::Point> target_points;  // 记录点到的所有目标区域

void point_callback(const geometry_msgs::PointStampedConstPtr point_msg)
{
    static int id_count = 0;
    target_points.push_back(tf::Point(point_msg->point.x, point_msg->point.y, 0.0));

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = id_count;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = point_msg->point.x;
    marker.pose.position.y = point_msg->point.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = region_radius;
    marker.scale.y = region_radius;
    marker.scale.z = 0.01;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration();

    marker_pub.publish(marker);

    id_count++;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "vlp16_ObstaclesDetection");
    ros::NodeHandle nh;
    ros::NodeHandle nh_param("~");

    nh_param.param<float>("detect_angle", detect_angle, M_PI_4);
    nh_param.param<float>("danger_range", danger_range, 0.5);
    nh_param.param<float>("region_radius", region_radius, 1.0);

    ros::Subscriber velodyne_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan1", 10, scan_callback);
    ros::Subscriber point_sub = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, point_callback);

    ros::Publisher region_pub = nh.advertise<std_msgs::Bool>("/in_region", 10);
    obstacles_pub = nh.advertise<std_msgs::Float32>("/obstacles_range", 10);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker",1);

    ros::Rate rate(10);

    while(ros::ok())
    {
        tf::StampedTransform tf_map2base;
        tf::TransformListener tf_listener;  // 此类型的变量不可以定义为全局变量

        TicToc tictoc;

        try {
            tf_listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(0.5));
            tf_listener.lookupTransform("/map", "/base_link", ros::Time(0), tf_map2base);
        }
        catch(tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
        }
        //cout << "time = " << tictoc.toc() << endl;

        tf::Point pose(tf_map2base.getOrigin().getX(), tf_map2base.getOrigin().getY(), 0.0);

        std_msgs::Bool in_region_msg;
        in_region_msg.data = false;

        // 计算当前位置与设定区域的距离
        for(auto &r : target_points) {
            double distance = pose.distance(r);
            ROS_INFO("distance = %f", distance);
            if(distance <= region_radius) {
                in_region_msg.data = true;
                ROS_WARN("Closing...");
                break;
            }
        }

        region_pub.publish(in_region_msg);

        //ROS_INFO("pose: %f, %f", pose.getX(), pose.getY());

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
