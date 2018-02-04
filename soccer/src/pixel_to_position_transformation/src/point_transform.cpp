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

float focal_length = 1000; 		// TODO get focal length from camera settings
float robot_height = 50; 		// TODO get robot height from robot info
float theta = 0;				// TODO angle of robot
float phi = M_PI / 9; 		// TODO angle looking down from robot head

// Distance
float D = 0.0;
geometry_msgs::Point P;
cv::Size2f camera_size;

void scale(geometry_msgs::Point& p, int scale) {
	p.x = p.x / scale;
	p.y = p.y / scale;
	p.z = p.z / scale;
}

geometry_msgs::Point get_center_point() {
	D = robot_height * tan(M_PI/2 - phi);

	float x = cos(theta) * D;
	float y = sin(theta) * D;

	P.x = x;
	P.y = y;
	P.z = 0;

	scale(P);
	return P;
}

geometry_msgs::Point point2d_to_3d(const geometry_msgs::Point point2d,
		float image_height, float image_width) {

	float x = (point2d.x - image_width / 2);
	float y = image_height / 2 - point2d.y;

	float D1 = D / cos(phi);

	float x_delta;

	float angle0 = atan(y / focal_length);
	float angle1 = (M_PI / 2 - phi) + angle0;
	float Dprime = tan(angle1) * robot_height;
	x_delta = Dprime - D;

	float Z1 = sqrt(pow((D + x_delta), 2) + pow(robot_height, 2));

	float y_delta = x / focal_length * Z1;

	geometry_msgs::Point p;

	float x1 = (D + x_delta);
	float y1 = (y_delta);

	p.x = x1 * cos(theta) - y1 * sin(theta);
	p.y = x1 * sin(theta) + y1 * cos(theta);
	p.z = 0;
	scale(p);

	return p;
}

int line_id = 1;

void draw_line(ros::Publisher& marker_pub, double xbot, double ybot,
		double xcomponent, double ycomponent, int steps) {
	visualization_msgs::Marker points, line_strip, line_list;
	points.header.frame_id = line_strip.header.frame_id =
			line_list.header.frame_id = "base_link";
	points.header.stamp = line_strip.header.stamp = line_list.header.stamp =
			ros::Time::now();
	points.ns = line_strip.ns = line_list.ns = "path_test";
	points.action = line_strip.action = line_list.action =
			visualization_msgs::Marker::ADD;
	points.pose.orientation.w = line_strip.pose.orientation.w =
			line_list.pose.orientation.w = 1.0;

	points.id = 0;
	line_strip.id = 1;
	line_list.id = line_id++;

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
	for (uint32_t i = 0; i < steps; ++i) {
		geometry_msgs::Point p;
		p.x = xbot + i * xcomponent;
		p.y = ybot + i * ycomponent;
		p.z = 0;

		points.points.push_back(p);
		line_strip.points.push_back(p);

		// The line list needs two points for each line
		line_list.points.push_back(p);
		line_list.points.push_back(p);
	}

	line_strip.lifetime = ros::Duration(5);

//	marker_pub.publish(points);
	marker_pub.publish(line_strip);
//	marker_pub.publish(line_list);
}

void draw_line(ros::Publisher& marker_pub, geometry_msgs::Point& start,
		geometry_msgs::Point& end, float steps) {
	draw_line(marker_pub, start.x, start.y, (end.x - start.x) / steps,
			(end.y - start.y) / steps, steps);
}

void draw_lines(ros::Publisher& marker_pub,
		humanoid_league_msgs::LineInformationRelative& lines, float steps) {

	visualization_msgs::Marker line_list;
	line_list.header.frame_id = "base_link";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "path_test";
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;

	line_list.id = 1;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.1;

	line_list.color.r = 1.0;
	line_list.color.a = 1.0;

	// Create the vertices for the points and lines
	for(auto it = lines.segments.begin(); it != lines.segments.end(); ++it) {
		line_list.points.push_back(it->start);
		line_list.points.push_back(it->end);
	}

	line_list.lifetime = ros::Duration(1);

	//	marker_pub.publish(points);
	marker_pub.publish(line_list);
}

void draw_point(ros::Publisher& marker_pub, geometry_msgs::Point& p) {
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

void draw_camera_border(ros::Publisher& marker_pub, cv::Size2f& camera_size) {
	humanoid_league_msgs::LineInformationRelative output;

	humanoid_league_msgs::LineSegmentRelative segment;

	geometry_msgs::Point p1;
	p1.x = 1;
	p1.y = 1;
	p1 = point2d_to_3d(p1, camera_size.height, camera_size.width);

	geometry_msgs::Point p2;
	p2.x = 1;
	p2.y = camera_size.height - 1;
	p2 = point2d_to_3d(p2, camera_size.height, camera_size.width);

	geometry_msgs::Point p3;
	p3.x = camera_size.width - 1;
	p3.y = 1;
	p3 = point2d_to_3d(p3, camera_size.height, camera_size.width);

	geometry_msgs::Point p4;
	p4.x = camera_size.width - 1;
	p4.y = camera_size.height - 1;
	p4 = point2d_to_3d(p4, camera_size.height, camera_size.width);

	segment.start = p1;
	segment.end = p2;
	output.segments.push_back(segment);

	segment.start = p1;
	segment.end = p3;
	output.segments.push_back(segment);

	segment.start = p2;
	segment.end = p4;
	output.segments.push_back(segment);

	segment.start = p3;
	segment.end = p4;
	output.segments.push_back(segment);

	draw_lines(marker_pub, output, 10);
}


void draw_camera_frame(ros::Publisher& marker_pub, cv::Size2f& camera_size) {
	visualization_msgs::Marker points;
	points.header.frame_id = "base_link";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;

	points.id = 0;

	points.type = visualization_msgs::Marker::POINTS;

	// POINTS markers use x and y scale for width/height respectively
	points.scale.x = 0.2;
	points.scale.y = 0.2;

	// Points are green
	points.color.g = 1.0f;
	points.color.a = 1.0;

	// Create the vertices for the points and lines
	float color = 0;
	for(int i = 1; i < camera_size.width - 1; i = i + 50) {
		for(int j = 1; j < camera_size.height - 1; j = j + 1) {
			geometry_msgs::Point p;
			p.x = i;
			p.y = j;
			geometry_msgs::Point p_3d = point2d_to_3d(p, camera_size.height, camera_size.width);
			points.points.push_back(p_3d);
		}
	}

	points.lifetime =  ros::Duration(1);

	marker_pub.publish(points);
}
