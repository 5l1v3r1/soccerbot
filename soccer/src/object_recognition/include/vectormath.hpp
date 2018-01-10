#ifndef VECTORMATH_HPP
#define VECTORMATH_HPP

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Polygon.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <string>
#include "camera.hpp"
#include "stdio.h"
#include "../include/statistics/kde.hpp"
#include <std_msgs/Int32.h>
#include <image_acquisition/SoccerColorSpace.h>

#define PI CV_PI
#define ANGLE_PROXIMITY_MIN PI / 72 // 10 degrees
#define ANGLE_PROXIMITY_MAX PI / 72 / 4 // 2 degrees
#define RHO_PROXIMITY_MIN 5
// 5 pixels

Point2f intersection(Vec2f a, Vec2f b);

Point2f leftScreenIntersection(Vec2f vec, Size2f imgSize);

Point2f rightScreenIntersection(Vec2f vec, Size2f imgSize);

bool isVerticalLine(Vec2f vec, Size2f imgSize);

vector<Vec2f> filterRepeats(vector<Vec2f>& vec);

vector<Vec2f> filterUnparallelRepeats(vector<Vec2f>& lines);

vector<Vec2f> filterByAngle(vector<Vec2f>& lines, float angleStart, float angleEnd);

void drawLinesOnImg(Mat& img, vector<Vec2f>& lines, Scalar color = Scalar(0, 255, 0));

#endif /* vectorfunction.hpp */
