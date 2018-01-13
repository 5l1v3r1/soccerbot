#include <ros/ros.h>
#include <ros/console.h>
#include <humanoid_league_msgs/Model.h>
#include <humanoid_league_msgs/GoalRelative.h>
#include <robot_control/WalkingPath.h>
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
class Listener{
public:
	int walkingpathind= 0;
	int hardwareind = 0;
	int turn1;
	int steps;
	int turn2;
	robot_control::WalkingPath walkingpath;
	std_msgs::String hardwareprev;
	void callback_path(const robot_control::WalkingPathConstPtr& msg);
	void callback_hardware(const std_msgs::StringConstPtr& msg);
};
void Listener::callback_path(const robot_control::WalkingPathConstPtr& msg) {
	ROS_INFO("Callback Path");
	walkingpathind = 1;
	walkingpath = *msg;
	turn1 = walkingpath.turns1;
	steps = walkingpath.steps;
	turn2 = walkingpath.turns2;
}

void Listener::callback_hardware(const std_msgs::StringConstPtr& msg) {
	ROS_INFO("Callback Hardware");
	if( (*msg).data == "Success" or (*msg).data == "Idle"){
		hardwareind = 1;
	}
	if( (*msg).data == "Failure"){
		hardwareind = 0;
		if(hardwareprev.data != "Idle"){
			hardwarecommand_publisher.publish(hardwareprev);
		}

	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Path Execution");
	ros::NodeHandle n;
	nh = &n;
	Listener listener;
	listener.hardwareprev.data = "Idle";
	path_subscriber = n.subscribe("/robot_control/WalkingPath", 1, &Listener::callback_path, &listener);
	hardwarecommand_subscriber = n.subscribe("/hardware_communication/hardware_callback", 1, &Listener::callback_hardware, &listener);
	hardwarecommand_publisher = n.advertise<std_msgs::String>("robot_control/execution", 1);


    ros::Rate r(2);
    while(ros::ok()) {
    	// Write you publish message here
    	if(listener.hardwareind==1 && listener.walkingpathind==1){
    		if(listener.turn1 !=0){
    			if (listener.turn1 < 0){
    				std_msgs::String msg;
    				msg.data= "Left Turn";
    				hardwarecommand_publisher.publish(msg);
    				listener.turn1++;
    				listener.hardwareprev = msg;
    			}
    			else{
    				std_msgs::String msg;
					msg.data = "Right Turn";
    				hardwarecommand_publisher.publish(msg);
    				listener.turn1--;
    				listener.hardwareprev = msg;
    			}
    		}
    		else if(listener.steps != 0){
    			std_msgs::String msg;
				msg.data = "Forward";
    			hardwarecommand_publisher.publish(msg);
    			listener.steps--;
    			listener.hardwareprev = msg;
    		}
    		else if(listener.turn2 != 0){
    			if (listener.turn2 < 0){
    				std_msgs::String msg;
					msg.data = "Left Turn";
					hardwarecommand_publisher.publish(msg);
					listener.turn2++;
					listener.hardwareprev = msg;
				}
				else{
					std_msgs::String msg;
					msg.data = "Right Turn";
					hardwarecommand_publisher.publish(msg);
					listener.turn2--;
					listener.hardwareprev = msg;
				};
    		}
    		else{
    			std_msgs::String msg;
    			msg.data = "Kick?";
    			hardwarecommand_publisher.publish(msg);
    			listener.hardwareprev = msg;
    		}
    	};

    	r.sleep();
    }

	ros::spin();
}
