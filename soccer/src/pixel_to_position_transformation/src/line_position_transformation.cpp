#include <ros/ros.h>
#include <iostream>
#include "stdio.h"
#include <humanoid_league_msgs/LineInformationInImage.h>
#include <humanoid_league_msgs/LineInformationRelative.h>
using namespace std;
using namespace ros;

Subscriber line_in_image;
Publisher line_relative;

void find_obstacle_position(const humanoid_league_msgs::LineInformationInImagePtr& msg) {

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "line_position_transformation");
    ros::NodeHandle n;

    line_in_image = n.subscribe("object_recognition/lines_in_image", 1, &find_obstacle_position);
    line_relative = n.advertise<humanoid_league_msgs::LineInformationRelative>("pixel_to_position_transformation/line_relative", 1);

    ros::spin();
}
