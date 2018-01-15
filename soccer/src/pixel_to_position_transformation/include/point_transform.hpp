/*
 * PointTransform.hpp
 *
 *  Created on: 2018-01-14
 *      Author: vuwij
 */

#ifndef PIXEL_TO_POSITION_TRANSFORMATION_INCLUDE_POINT_TRANSFORM_HPP_
#define PIXEL_TO_POSITION_TRANSFORMATION_INCLUDE_POINT_TRANSFORM_HPP_

// Distances in mm
#include "../include/point_transform.hpp"
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>

// Obtained using robot information
extern float focal_length; 		// TODO get focal length from camera settings
extern float robot_height; 		// TODO get robot height from robot info
extern float theta;				// TODO angle of robot
extern float phi; 				// TODO angle looking down from robot head


extern float D;

extern cv::Size2f camera_size;

geometry_msgs::Point point2d_to_3d(const geometry_msgs::Point point2d, float image_height, float image_width);

geometry_msgs::Point get_center_point();

void draw_line(ros::Publisher& marker_pub, double xbot, double ybot, double xcomponent, double ycomponent, int steps);

void draw_point(ros::Publisher& marker_pub, geometry_msgs::Point& p);

#endif /* PIXEL_TO_POSITION_TRANSFORMATION_INCLUDE_POINT_TRANSFORM_HPP_ */
