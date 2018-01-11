#include <ros/ros.h>
#include <ros/console.h>
#include <humanoid_league_msgs/Model.h>
#include <humanoid_league_msgs/GoalRelative.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>

using namespace std;
using namespace ros;

// Publisher Subscribers
ros::NodeHandle* nh;
ros::Subscriber path_subscriber;
ros::Subscriber hardwarecommand_subscriber;
ros::Publisher hardwarecommand_publisher;

int image_count = 0;

void callback_path(const nav_msgs::PathConstPtr& msg) {
	ROS_INFO("Callback Path");
}

void callback_hardware(const std_msgs::String& msg) {
	ROS_INFO("Callback Hardware");
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Path Execution");
	ros::NodeHandle n;
	nh = &n;

	path_subscriber = n.subscribe("/robot_control/path", 1, callback_path);
	hardwarecommand_subscriber = n.subscribe("/hardware_communication/hardware_callback", 1, callback_hardware);
	hardwarecommand_publisher = n.advertise<std_msgs::String>("robot_control/execution", 1);


    ros::Rate r(2);
    while(ros::ok()) {
    	// Write you publish message here

    	r.sleep();
    }

	ros::spin();
}
