#include <ros/ros.h>
#include <ros/console.h>
#include <humanoid_league_msgs/Model.h>
#include <humanoid_league_msgs/GoalRelative.h>
#include <robot_control/WalkingPath.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace ros;

// Publisher Subscribers
ros::NodeHandle* nh;
ros::Subscriber model_subscriber;
ros::Subscriber goal_relative_subscriber;
ros::Publisher path_publisher;
ros::Publisher marker_pub;
ros::Publisher goal_pub;

int image_count = 0;
const double KICK_DISTANCE = 0.5; //meters
const double TURN_SPEED = 0.1; //radians
const double WALK_SPEED = 0.1; //meters
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
	checkmodel = 1;
	model = *msg;
}

void Listener::callback_goalrelative(const humanoid_league_msgs::GoalRelativeConstPtr& msg) {
	ROS_INFO("Callback Goal Relative");
	checkgoal = 1;
	goal = *msg;
};

void draw_line(ros::Publisher& marker_pub, double xbot, double ybot, double xcomponent, double ycomponent, int steps){
	visualization_msgs::Marker points, line_strip, line_list;
	points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "base_link";
	points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
	points.ns = line_strip.ns = line_list.ns = "path_test";
	points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;



	points.id = 0;
	line_strip.id = 1;
	line_list.id = 2;



	points.type = visualization_msgs::Marker::POINTS;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	line_list.type = visualization_msgs::Marker::LINE_LIST;



	// POINTS markers use x and y scale for width/height respectively
	points.scale.x = 0.2;
	points.scale.y = 0.2;

	// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
	line_strip.scale.x = 0.1;
	line_list.scale.x = 0.1;



	// Points are green
	points.color.g = 1.0f;
	points.color.a = 1.0;

	// Line strip is blue
	line_strip.color.b = 1.0;
	line_strip.color.a = 1.0;

	// Line list is red
	line_list.color.r = 1.0;
	line_list.color.a = 1.0;



	// Create the vertices for the points and lines
	for (uint32_t i = 0; i < steps; ++i)
	{
	  geometry_msgs::Point p;
	  p.x = xbot+i*xcomponent;
	  p.y = ybot+i*ycomponent;
	  p.z = 0;

	  points.points.push_back(p);
	  line_strip.points.push_back(p);

	  // The line list needs two points for each line
	  line_list.points.push_back(p);
	  line_list.points.push_back(p);
	}


	marker_pub.publish(points);
	marker_pub.publish(line_strip);
	marker_pub.publish(line_list);
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
	double distance = sqrt((truexball-xbot)*(truexball-xbot)+(trueyball-ybot)*(trueyball-ybot));
	double alpha = atan((trueyball-ybot)/(truexball-xbot)); //orientation to approach the ball
	double w = model.position.pose.pose.orientation.w;

	int turns1 = (int)(alpha-w)/TURN_SPEED;
	//add direction

	int steps = (int)distance/WALK_SPEED;
	double xcomponent = (truexball-xbot)/steps;
	double ycomponent = (trueyball-ybot)/steps;
	int turns2 = (int)(beta-alpha)/TURN_SPEED;

	int vis_step = 5;
	double xcompgoal = (xgoal-xball)/vis_step;
	double ycompgoal = (ygoal-yball)/vis_step;
	robot_control::WalkingPath output;
	output.turns1 = turns1;
	output.steps = steps;
	output.turns2 = turns2;

	draw_line(marker_pub, xbot, ybot, xcomponent, ycomponent, steps);
	draw_line(goal_pub, xball, yball, xcompgoal, ycompgoal, vis_step);
	return output;
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "Path Planning");
	ros::NodeHandle n;
	nh = &n;
	Listener listener;
    model_subscriber = n.subscribe("/localization/model", 1, &Listener::callback_model, &listener);
    goal_relative_subscriber = n.subscribe("/localization/goal_relative", 1, &Listener::callback_goalrelative, &listener);
    path_publisher = n.advertise<robot_control::WalkingPath>("/robot_control/WalkingPath", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    goal_pub = n.advertise<visualization_msgs::Marker>("visualization_goal_marker", 10);
    ros::Rate r(1);

    while(ros::ok()) {
    	// Write you publish message heres
    	if(listener.checkmodel==1 && listener.checkgoal==1 ){
    		robot_control::WalkingPath msg = listener.computedestination(listener.model, listener.goal);
    		ROS_INFO("Ready to launch");
    		path_publisher.publish(msg);
    	};
    	ros::spinOnce();
    	r.sleep();
    }

	ros::spin();
}
