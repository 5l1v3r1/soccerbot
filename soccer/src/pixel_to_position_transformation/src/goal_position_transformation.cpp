#include <ros/ros.h>
#include <iostream>
#include "stdio.h"
#include <humanoid_league_msgs/GoalInImage.h>
#include <humanoid_league_msgs/GoalRelative.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "../include/point_transform.hpp"

using namespace std;
using namespace ros;

Subscriber obstacle_in_image;
Publisher goal_relative;

void find_obstacle_position(const humanoid_league_msgs::GoalInImagePtr& msg) {
	geometry_msgs::Point p1 = msg->left_post.foot_point;
	geometry_msgs::Point p2 = msg->right_post.foot_point;
	geometry_msgs::Point p1_3d = point2d_to_3d(p1, camera_size.height, camera_size.width);
	geometry_msgs::Point p2_3d = point2d_to_3d(p2, camera_size.height, camera_size.width);

	humanoid_league_msgs::GoalRelative output;
	output.left_post = p1_3d;
	output.right_post = p2_3d;
	goal_relative.publish(output);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "goal_position_transformation");
    ros::NodeHandle n;

    obstacle_in_image = n.subscribe("object_recognition/goal_in_image", 1, &find_obstacle_position);
    goal_relative = n.advertise<humanoid_league_msgs::GoalRelative>("pixel_to_position_transformation/goal_relative", 1);

    ros::spin();
}
