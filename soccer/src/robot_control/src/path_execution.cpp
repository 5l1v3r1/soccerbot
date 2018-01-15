#include <ros/ros.h>
#include <ros/console.h>
#include <humanoid_league_msgs/Model.h>
#include <humanoid_league_msgs/GoalRelative.h>
#include <robot_control/WalkingPath.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <hardware_communication/RobotGoal.h>
#include <hardware_communication/RobotState.h>

using namespace std;
using namespace ros;

// Publisher Subscribers
ros::NodeHandle* nh;
ros::Subscriber path_subscriber;
ros::Subscriber hardwarecommand_subscriber;
ros::Publisher hardwarecommand_publisher;

enum MoveType {
	turnLeft,
	turnRight,
	moveForward,
	success,
	failure,
	kick,
	idle,
	waiting
};

class Listener {
public:
	int walkingpathind= 0;
	int hardwareind = 1;
	int turn1;
	int steps;
	int turn2;


	MoveType type;
	robot_control::WalkingPath walkingpath;
	hardware_communication::RobotState hardwareprev;
	void callback_path(const robot_control::WalkingPathConstPtr& msg);
	void callback_hardware(const hardware_communication::RobotStateConstPtr& msg);
};
void Listener::callback_path(const robot_control::WalkingPathConstPtr& msg) {
	ROS_INFO("Callback Path");
	walkingpathind = 1;
	walkingpath = *msg;
	turn1 = walkingpath.turns1;
	steps = walkingpath.steps;
	turn2 = walkingpath.turns2;
	type = MoveType::turnLeft;
}

void Listener::callback_hardware(const hardware_communication::RobotStateConstPtr& msg) {
	ROS_INFO("Callback Hardware");
	if( (*msg).message == MoveType::success || (*msg).message == MoveType::idle){
		hardwareind = 1;
	}
	else if( (*msg).message == MoveType::failure){
		hardwareind = 0;
		if(hardwareprev.message != MoveType::idle){
			hardwarecommand_publisher.publish(hardwareprev);
		}

	}
	else if( (*msg).message == MoveType::waiting){
		hardwareind = 0;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Path Execution");
	ros::NodeHandle n;
	nh = &n;
	Listener listener;
	listener.hardwareprev.message = MoveType::idle;
	path_subscriber = n.subscribe("/robot_control/WalkingPath", 1, &Listener::callback_path, &listener);
	hardwarecommand_subscriber = n.subscribe("/hardware_communication/RobotState", 1, &Listener::callback_hardware, &listener);
	hardwarecommand_publisher = n.advertise<hardware_communication::RobotState>("/robot_control/execution", 1);


    ros::Rate r(1);
    while(ros::ok()) {
    	hardware_communication::RobotState msg;
    	msg.message= MoveType::turnLeft;
    	hardwarecommand_publisher.publish(msg);
    	// Write you publish message here
    	if (listener.walkingpathind == 0){
    		ROS_INFO("No walking path");
    	};
    	ROS_INFO("Walking path");
    	if(listener.hardwareind==1 && listener.walkingpathind==1){
    		ROS_INFO("Deciding optimal move");
    		listener.hardwareind = 0;
    		if(listener.turn1 != 0){
    			if (listener.turn1 < 0){
    				ROS_INFO("turnleft");
    				hardware_communication::RobotState msg;
    				msg.message= MoveType::turnLeft;
    				hardwarecommand_publisher.publish(msg);
    				listener.turn1++;
    				listener.hardwareprev = msg;
    			}
    			else{
    				ROS_INFO("turnright");
    				hardware_communication::RobotState msg;
					msg.message = MoveType::turnRight;
    				hardwarecommand_publisher.publish(msg);
    				listener.turn1--;
    				listener.hardwareprev = msg;
    			}
    		}
    		else if(listener.steps != 0){
    			ROS_INFO("forward");
    			hardware_communication::RobotState msg;
				msg.message = MoveType::moveForward;
    			hardwarecommand_publisher.publish(msg);
    			listener.steps--;
    			listener.hardwareprev = msg;
    		}
    		else if(listener.turn2 != 0){
    			if (listener.turn2 < 0){
    				ROS_INFO("turnleft2");
    				hardware_communication::RobotState msg;
					msg.message = MoveType::turnLeft;
					hardwarecommand_publisher.publish(msg);
					listener.turn2++;
					listener.hardwareprev = msg;
				}
				else{
					ROS_INFO("turnright2");
					hardware_communication::RobotState msg;
					msg.message = MoveType::turnRight;
					hardwarecommand_publisher.publish(msg);
					listener.turn2--;
					listener.hardwareprev = msg;
				};
    		}
    		else{
    			hardware_communication::RobotState msg;
    			msg.message = MoveType::kick;
    			hardwarecommand_publisher.publish(msg);
    			listener.hardwareprev = msg;
    		}
    	};

    	ros::spinOnce();
    	r.sleep();
    }
}
