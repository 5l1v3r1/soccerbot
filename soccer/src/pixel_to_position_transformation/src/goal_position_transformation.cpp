#include <ros/ros.h>
#include <iostream>
#include "stdio.h"
#include <humanoid_league_msgs/GoalInImage.h>
#include <humanoid_league_msgs/GoalRelative.h>
using namespace std;
using namespace ros;

Subscriber obstacle_in_image;
Publisher goal_relative;

void find_obstacle_position(const humanoid_league_msgs::GoalInImagePtr& msg) {

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "goal_position_transformation");
    ros::NodeHandle n;

    obstacle_in_image = n.subscribe("object_recognition/goal_in_image", 1, &find_obstacle_position);
    goal_relative = n.advertise<humanoid_league_msgs::GoalRelative>("pixel_to_position_transformation/goal_relative", 1);

    ros::spin();
}
