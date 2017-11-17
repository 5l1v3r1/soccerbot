#include <ros/ros.h>
#include <iostream>
#include "stdio.h"
#include <humanoid_league_msgs/ObstacleInImage.h>
#include <humanoid_league_msgs/ObstacleRelative.h>
using namespace std;
using namespace ros;

Subscriber obstacle_in_image;
Publisher obstacle_relative;

void find_obstacle_position(const humanoid_league_msgs::ObstacleInImagePtr& msg) {

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "opponent_position_transformation");
    ros::NodeHandle n;

    obstacle_in_image = n.subscribe("object_recognition/obstacles_in_image", 1, &find_obstacle_position);
    obstacle_relative = n.advertise<humanoid_league_msgs::ObstacleRelative>("pixel_to_position_transformation/obstacles_relative", 1);

    ros::spin();
}
