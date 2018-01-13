#include <ros/ros.h>
#include <ros/console.h>
#include <humanoid_league_msgs/Model.h>
#include <humanoid_league_msgs/GoalRelative.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>

using namespace std;
using namespace ros;

// Publisher Subscribers
ros::NodeHandle* nh;
ros::Publisher model_publisher;
ros::Publisher goal_relative_publisher;
ros::Publisher hardware_publisher;
ros::Publisher pose_publisher;
ros::Publisher bad_publisher;
ros::Subscriber hardware_subscriber;

void test() {
	ROS_INFO("Testing");
	humanoid_league_msgs::Model m;

	// Ball
	m.ball.ball_relative.x = 3;
	m.ball.ball_relative.y = 3;
	m.ball.confidence = 0;

	// Person
	m.position.pose.pose.position.x = 1;
	m.position.pose.pose.position.y = 1;
	m.position.pose.pose.orientation.w = 0;

	humanoid_league_msgs::GoalRelative g;
	g.center_direction.x = 5;
	g.center_direction.y = 5;

	model_publisher.publish(m);
	goal_relative_publisher.publish(g);

	// Draw the current coordinate in RVIZ

	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time();
	marker.ns = "path_test";
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = m.ball.ball_relative.x;
	marker.pose.position.y = m.ball.ball_relative.y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 0;
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;
	marker.id = 0;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	marker.lifetime = ros::Duration(1);
	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	pose_publisher.publish(marker);

	// Draw the current location

	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time();
	marker.ns = "path_test";
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = m.position.pose.pose.position.x;
	marker.pose.position.y = m.position.pose.pose.position.y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 0;
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;
	marker.id = 0;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	marker.lifetime = ros::Duration(1);
	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	bad_publisher.publish(marker);


}
void callback_hardware(const std_msgs::StringConstPtr& msg){
	ROS_INFO("Callback hardware");
	std_msgs::String output;
	output.data = "Success";
	hardware_publisher.publish(output);
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "Path Test");
	ros::NodeHandle n;
	nh = &n;

    model_publisher = n.advertise<humanoid_league_msgs::Model>("/localization/model",1);
    goal_relative_publisher = n.advertise<humanoid_league_msgs::GoalRelative>("/localization/goal_relative", 1);
    hardware_publisher = n.advertise<std_msgs::String>("/hardware_communication/hardware_callback", 1);
    hardware_subscriber = n.subscribe("/robot_control/execution",1, callback_hardware);
    pose_publisher = n.advertise<visualization_msgs::Marker>("Ball_Marker", 0);
    bad_publisher = n.advertise<visualization_msgs::Marker>("Position_Marker", 0);

    ros::Rate r(1);

    while(ros::ok()) {
    	// Write you publish message here
    	test();

    	ros::spinOnce();
    	r.sleep();
    }

	ros::spin();
}
