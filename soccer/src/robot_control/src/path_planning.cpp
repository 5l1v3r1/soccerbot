#include <ros/ros.h>
#include <ros/console.h>
#include <humanoid_league_msgs/Model.h>
#include <humanoid_league_msgs/GoalRelative.h>
#include <nav_msgs/Path.h>

using namespace std;
using namespace ros;

// Publisher Subscribers
ros::NodeHandle* nh;
ros::Subscriber model_subscriber;
ros::Subscriber goal_relative_subscriber;
ros::Publisher path_publisher;

int image_count = 0;

humanoid_league_msgs::Model model;

void callback_model(const humanoid_league_msgs::ModelConstPtr& msg) {
	ROS_INFO("Callback Model");
}

void callback_goalrelative(const humanoid_league_msgs::GoalRelativeConstPtr& msg) {
	ROS_INFO("Callback Goal Relative");
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Path Planning");
	ros::NodeHandle n;
	nh = &n;

    model_subscriber = n.subscribe("/localization/model", 1, callback_model);
    goal_relative_subscriber = n.subscribe("/localization/goal_relative", 1, callback_goalrelative);
    path_publisher = n.advertise<nav_msgs::Path>("/robot_control/path", 1);

    ros::Rate r(1000);

    while(ros::ok()) {
    	// Write you publish message heres

    	ros::spinOnce();
    	r.sleep();
    }

	ros::spin();
}
