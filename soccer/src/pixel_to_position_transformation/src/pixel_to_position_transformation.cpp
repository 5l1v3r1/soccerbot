#include <ros/ros.h>
#include <iostream>
#include "stdio.h"
#include <visualization_msgs/Marker.h>

#include "../include/point_transform.hpp"

using namespace std;
using namespace ros;

ros::Publisher center_point;

int main(int argc, char **argv) {

    ros::init(argc, argv, "pixel_to_position_transformation");
    ros::NodeHandle n;

    center_point = n.advertise<visualization_msgs::Marker>("/pixel_to_position_transformation/center_point", 1);

    ros::Rate r(30);
    while(ros::ok()) {
    	geometry_msgs::Point m = get_center_point(); // TODO Publish this
    	draw_point(center_point, m);

    	r.sleep();
    	ros::spin();
    }
}
