#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Int32.h>   //temporary
#include <team_communication/team_data.h>
//#include <behaviour/strategy.h>
//#include <robot_control/robot_state.h>

using namespace std;

ros::Publisher pub;

void callbackFunction(const std_msgs::Int32::ConstPtr& msg)
{
	//emp
}

void callbackFunction2(const std_msgs::Int32::ConstPtr& msg)  //behaviour::strategy::ConstPtr& msg
{
	//emp
}

void callbackFunction3(const std_msgs::Int32::ConstPtr& msg)  //robot_control::robot_state::ConstPtr& msg
{
	//emp
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "team_communication");
    ros::NodeHandle n;
		
	// increase the capacity of each if needed
    pub = n.advertise<team_communication::team_data>("team_data", 1);
    ros::Subscriber sub = n.subscribe("personal_model", 10, callbackFunction); //mod topic names later
    ros::Subscriber sub2 = n.subscribe("strategy", 10, callbackFunction2);
    ros::Subscriber sub3 = n.subscribe("robot_state", 10, callbackFunction3);
    
}
