/*
 * PointTransform.hpp
 *
 *  Created on: 2018-01-14
 *      Author: vuwij
 */

#include "../include/point_transform.hpp"
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>

using namespace std;

float focal_length = 50; 		// TODO get focal length from camera settings
float robot_height = 50; 		// TODO get robot height from robot info
float theta = 0;				// TODO angle of robot
float phi = - M_PI / 12; 		// TODO angle looking down from robot head

// Distance
float D = 0.0;
geometry_msgs::Point P;
cv::Size2f camera_size;

geometry_msgs::Point get_center_point() {
	D = robot_height / tan(phi);

	float x = sin(theta) * D;
	float y = sin(theta) * D;

	P.x = x;
	P.y = y;
	P.z = 0;

	return P;
}


geometry_msgs::Point point2d_to_3d(const geometry_msgs::Point point2d, float image_height, float image_width) {

	float x = point2d.x - image_width / 2;
	float y = point2d.y - image_height / 2;

	float D1 = D /  cos(phi);

	float y_delta = (y / focal_length * D1) / cos(M_PI/2 - phi);
	float Z1 = sqrt(pow((y_delta + D),2) + pow(robot_height,2));

	float x_delta = x / focal_length * Z1;

	geometry_msgs::Point p;
	p.x = P.x + x_delta;
	p.y = P.y + y_delta;
	p.z = 0;

	return p;
}

void draw_line(ros::Publisher& marker_pub, double xbot, double ybot, double xcomponent, double ycomponent, int steps) {
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

//	marker_pub.publish(points);
	marker_pub.publish(line_strip);
//	marker_pub.publish(line_list);
}

void draw_point (ros::Publisher& marker_pub, geometry_msgs::Point& p) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time();
	marker.ns = "path_test";
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = p.x;
	marker.pose.position.y = p.y;
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
	marker_pub.publish(marker);
}
