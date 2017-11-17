#include <ros/ros.h>
#include <iostream>
#include "stdio.h"
#include <humanoid_league_msgs/BallInImage.h>
#include <humanoid_league_msgs/BallRelative.h>
using namespace std;
using namespace ros;

Subscriber ball_in_image;
Publisher ball_relative;

void find_ball_in_image(const humanoid_league_msgs::BallInImagePtr& msg) {

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "ball_position_transformation");
    ros::NodeHandle n;

    ball_in_image = n.subscribe("object_recognition/ball_in_image", 1, &find_ball_in_image);
    ball_relative = n.advertise<humanoid_league_msgs::BallRelative>("pixel_to_position_transformation/ball_relative", 1);

    ros::spin();
}
