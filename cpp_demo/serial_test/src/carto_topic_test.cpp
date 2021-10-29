#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <stdio.h>

class sub_pub
{
public:
    sub_pub()
    {
        sub = nh.subscribe<sensor_msgs::LaserScan>("/test_scan", 1000, &sub_pub::subCallback,this);
        pub_laser = nh.advertise<sensor_msgs::LaserScan>("/scan", 1000);
    }

    void subCallback(const sensor_msgs::LaserScanConstPtr& msg)
    {
        ros::Time time = ros::Time::now();
        sensor_msgs::LaserScan pub_scan = *msg;
        pub_scan.header.stamp = time;
        pub_laser.publish(pub_scan);  
        printf("ok!\n");
    }

public:
    ros::Subscriber sub;
    ros::NodeHandle nh;
    ros::Publisher pub_laser;
};


int main(int argc,char** argv)
{
    ros::init(argc, argv, "carto_topic_test");
    
    sub_pub carto_test;
    tf::TransformBroadcaster br;
    tf::Transform transform1;
    while(ros::ok())
    {
        ros::spinOnce();
    }
}