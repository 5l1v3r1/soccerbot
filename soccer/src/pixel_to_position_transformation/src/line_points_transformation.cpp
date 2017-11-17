#include <ros/ros.h>
#include <iostream>
#include "stdio.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
using namespace std;
using namespace ros;

Subscriber line_points_in_image;
Publisher line_points_relative;

void find_obstacle_position(const sensor_msgs::PointCloud2Ptr& msg) {

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "line_points_transformation");
    ros::NodeHandle n;

    line_points_in_image = n.subscribe("object_recognition/line_points_in_image", 1, &find_obstacle_position);
    line_points_relative = n.advertise<sensor_msgs::PointCloud>("pixel_to_position_transformation/line_points_relative", 1);

    ros::spin();
}
