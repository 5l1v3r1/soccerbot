#include <ros/ros.h>
#include <ros/console.h>
#include <humanoid_league_msgs/Model.h>
#include <humanoid_league_msgs/GoalRelative.h>
#include <nav_msgs/Path.h>

using namespace std;
using namespace ros;

// Publisher Subscribers
ros::NodeHandle* nh;
ros::Publisher model_publisher;
ros::Publisher goal_relative_publisher;

void test() {
	ROS_INFO("Testing");
	humanoid_league_msgs::Model m;
	m.ball.confidence = 0;
	model_publisher.publish(m);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Path Test");
	ros::NodeHandle n;
	nh = &n;

    model_publisher = n.advertise<humanoid_league_msgs::Model>("/localization/model",1);
//    goal_relative_publisher = n.advertise<humanoid_league_msgs::GoalRelative>("/localization/goal_relative", 1);

    ros::Rate r(2);

    while(ros::ok()) {
    	// Write you publish message here
    	test();

    	ros::spinOnce();
    	r.sleep();
    }

	ros::spin();
}
