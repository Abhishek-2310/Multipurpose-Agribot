#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

using namespace std;

float x = 0.0;
float y = 0.0;
float z = 0.0;

void counterCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;
    // ROS_INFO("aneesh");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "seeder");

    ros::NodeHandle nh;
    ros::Publisher pos_pub = nh.advertise<std_msgs::Float64>("/agribot_v2/seeder_joint_position_controller/command", 1);
    ros::Publisher cmdvel_pub = nh.advertise<geometry_msgs::Twist>("/agribot/mobile_base_controller/cmd_vel", 1);

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber odom_sub = nh.subscribe("/agribot/mobile_base_controller/odom", 10, counterCallback);

    visualization_msgs::Marker points;
    points.id = 0;
    points.header.frame_id = "odom";
    points.ns = "seeder";
    points.header.stamp = ros::Time::now();
    points.action = visualization_msgs::Marker::ADD;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    points.scale.z = 0.2;
    points.pose.orientation.w = 1.0;
    points.color.r = 0.0f;
    points.color.g = 1.0f;
    points.color.b = 0.0f;
    points.color.a = 1.0f;

    // ros::Rate loopRate(100);
    
    ros::Time previousTime = ros::Time::now();
	
	// 100 ms delay
    ros::Duration d = ros::Duration(0, 100000000);
    std_msgs::Float64 pos;
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.5;

    while (ros::ok())
    {
        ros::Time currentTime = ros::Time::now();

        if(currentTime.sec - previousTime.sec >= 2)
        {
            previousTime = currentTime;
            pos.data = -1.0;
            pos_pub.publish(pos);

            geometry_msgs::Point p;
            p.x = x;
            p.y = y - 0.2;
            p.z = z;
            points.points.push_back(p);

            ROS_INFO("seeding started");
            d.sleep();
            ROS_INFO("seeding ended");

            marker_pub.publish(points);
        }
        else
        {
            pos.data = 1.0;
            pos_pub.publish(pos);
        }
        
        cmdvel_pub.publish(cmd);
        
        ros::spinOnce();
    }

    return 0;
}
