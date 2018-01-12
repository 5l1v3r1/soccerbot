#include <ros/ros.h>
#include <ros/console.h>
#include <humanoid_league_msgs/Model.h>
#include <humanoid_league_msgs/GoalRelative.h>
#include <robot_control/WalkingPath.h>
#include <geometry_msgs/Point.h>
#include <math.h>

using namespace std;
using namespace ros;

// Publisher Subscribers
ros::NodeHandle* nh;
ros::Subscriber model_subscriber;
ros::Subscriber goal_relative_subscriber;
ros::Publisher path_publisher;

int image_count = 0;
const double KICK_DISTANCE = 5; //meters
const double TURN_SPEED = 1; //radians
const double WALK_SPEED = 1; //meters
humanoid_league_msgs::Model model;
class Listener {
public:
	int checkmodel = 0;
	int checkgoal = 0;
	humanoid_league_msgs::Model model;
	humanoid_league_msgs::GoalRelative goal;
	void callback_model(const humanoid_league_msgs::ModelConstPtr& msg);
	void callback_goalrelative(const humanoid_league_msgs::GoalRelativeConstPtr& msg);
	robot_control::WalkingPath computedestination(const humanoid_league_msgs::Model model, const humanoid_league_msgs::GoalRelative goal);
};

void Listener::callback_model(const humanoid_league_msgs::ModelConstPtr& msg) {
	ROS_INFO("Callback Model");
	checkmodel = 0;
	model = *msg;
}

void Listener::callback_goalrelative(const humanoid_league_msgs::GoalRelativeConstPtr& msg) {
	ROS_INFO("Callback Goal Relative");
	checkgoal = 0;
	goal = *msg;
};

robot_control::WalkingPath Listener::computedestination(humanoid_league_msgs::Model model, humanoid_league_msgs::GoalRelative goal){
	ROS_INFO("Computing");
	double xball = model.ball.ball_relative.x;
	double yball = model.ball.ball_relative.y;
	double xbot = model.position.pose.pose.position.x;
	double ybot = model.position.pose.pose.position.y;
	//figure out quaternions later
	double xgoal = goal.center_direction.x;
	double ygoal = goal.center_direction.y;

	double beta = atan((ygoal-yball)/(xgoal-xball)); //orientation to shoot the ball

	double truexball = xball-KICK_DISTANCE*cos(beta);
	double trueyball = yball-KICK_DISTANCE*sin(beta);
	double distance = (truexball-xbot)*(truexball-xbot);
	double alpha = atan((trueyball-ybot)/(truexball-xbot)); //orientation to approach the ball
	double w = model.position.pose.pose.orientation.w;

	int turns1 = (alpha-w)/TURN_SPEED;
	//add direction
	int steps = distance/WALK_SPEED;
	int turns2 = (beta-alpha)/TURN_SPEED;
	robot_control::WalkingPath output;
	output.turns1 = turns1;
	output.steps = steps;
	output.turns2 = turns2;
	return output;
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "Path Planning");
	ros::NodeHandle n;
	nh = &n;
	Listener listener;
    model_subscriber = n.subscribe("/localization/model", 1, &Listener::callback_model, &listener);
    goal_relative_subscriber = n.subscribe("/localization/goal_relative", 1, &Listener::callback_goalrelative, &listener);
    path_publisher = n.advertise<robot_control::WalkingPath>("/robot_control/path", 1);

    ros::Rate r(1000);

    while(ros::ok()) {
    	// Write you publish message heres
    	if(listener.checkmodel==1 && listener.checkgoal==1 ){
    		ROS_INFO("Ready to launch");
    		robot_control::WalkingPath msg = listener.computedestination(listener.model, listener.goal);
    		path_publisher.publish(msg);
    	};
    	ros::spinOnce();
    	r.sleep();
    }

	ros::spin();
}
