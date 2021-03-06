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
#include <vectormath.hpp>
#include <object_recognition/ROI.h>
#include <humanoid_league_msgs/LineInformationInImage.h>
//#include <object_recognition/FieldBoundary.h>

using namespace std;
using namespace ros;
using namespace cv;

// Publisher Subscribers
ros::NodeHandle* nh;
ros::Publisher field_roi;
ros::Publisher field_border;
image_transport::Publisher field_area_img;
image_transport::Subscriber hsv_img;

// Constants
Scalar lower = Scalar(40, 0, 0);
Scalar upper = Scalar(60, 255, 200);

int image_count = 0;

void find_field_area(const sensor_msgs::ImageConstPtr& msg) {
	ROS_INFO("Field Area");
	cv_bridge::CvImageConstPtr img;
	try {
		img = cv_bridge::toCvShare(msg, "");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	Mat mask, mask2, mask3;
	inRange(img->image, lower, upper, mask);

	int erosion_size = 17;
	Mat element = getStructuringElement(MORPH_ELLIPSE,
			Size(2 * erosion_size + 1, 2 * erosion_size + 1),
			Point(erosion_size, erosion_size));

	int erosion_size2 = 13;
	Mat element2 = getStructuringElement(MORPH_ELLIPSE,
			Size(2 * erosion_size2 + 1, 2 * erosion_size2 + 1),
			Point(erosion_size2, erosion_size2));

	int erosion_size3 = 3;
	Mat element3 = getStructuringElement(MORPH_ELLIPSE,
			Size(2 * erosion_size + 1, 2 * erosion_size + 1),
			Point(erosion_size, erosion_size));

	// Dilate and Erode for small parts
	dilate(mask, mask2, element);
	erode(mask2, mask2, element);
	bitwise_not(mask2, mask3);

	dilate(mask3, mask3, element);
	erode(mask3, mask3, element);
	bitwise_not(mask3, mask3);


	double minarea = ((double) (img->image.rows * img->image.cols) / 30);
	double tmparea = 0.0;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(mask3, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
	for (size_t i = 0; i < contours.size(); i++) {
		tmparea = contourArea(contours[i]);
		if (tmparea < minarea) {
			drawContours(mask3, contours, i, CV_RGB(0, 0, 0), CV_FILLED);
		}
	}
	bitwise_not(mask3, mask3);
	findContours(mask3, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
	for (size_t i = 0; i < contours.size(); i++) {
		tmparea = contourArea(contours[i]);
		if (tmparea < minarea) {
			drawContours(mask3, contours, i, CV_RGB(0, 0, 0), CV_FILLED);
		}
	}
	bitwise_not(mask3, mask3);

	dilate(mask3, mask3, element2);

	// Find the polygon information
	Mat edges, cedges;
	Canny(mask3, edges, 50, 200, 3);
	cvtColor(edges, cedges, CV_GRAY2BGR);

	vector<Vec2f> lines;
	HoughLines(edges, lines, 1, CV_PI / 180, 40, 0, 0);

	Mat field_area_mat;
	img->image.copyTo(field_area_mat, mask3);
	if(lines.size() != 0) {

		// Go through the vertical lines and find ones that are not straight
		vector<Vec2f> valid_lines;
		for(auto it = lines.begin(); it != lines.end(); ++it) {
			if(isVerticalLine(*it, img->image.size())) continue;
			if((*it)[0] > img->image.size().height) continue;
			valid_lines.push_back(*it);
		}
		vector<Vec2f> peaks = filterRepeats(valid_lines);
//		for(auto it = peaks.begin(); it < peaks.end(); ++it) {
//			ROS_INFO("%d %f %f",image_count, (*it)[0], (*it)[1]);
//		}

		Mat imtest;
		cvtColor(img->image, imtest, cv::COLOR_HSV2BGR);
		drawLinesOnImg(cedges, valid_lines, Scalar(0, 255, 0));
		drawLinesOnImg(cedges, peaks, Scalar(0, 0, 255));

		// Extract the intersections of the line
		vector<Point2f> intersections;
		for(int i = 0; i < peaks.size() - 1; ++i) {
			Point2f intersect = intersection(peaks[i], peaks[i+1]);
//			ROS_ERROR("%f %f", intersect.x, intersect.y);
			intersections.push_back(intersect);
			circle(cedges, intersect, 5, Scalar(0,255,0));
		}
		//ROS_ERROR("%d %d", img->image.size().height, img->image.size().width);
		Point2f right_int = rightScreenIntersection(peaks[peaks.size() - 1], img->image.size());
		//ROS_ERROR("%f %f", right_int.x, right_int.y);
		circle(cedges, right_int, 5, Scalar(0,255,0));

		Point2f left_int = leftScreenIntersection(peaks[0], img->image.size());
		//ROS_ERROR("%f %f", left_int.x, left_int.y);
		circle(cedges, left_int, 5, Scalar(0,255,0));

		vector<Point2f> points, pointsinv;
		points.push_back(left_int);
		pointsinv.push_back(left_int);
		for (auto it = intersections.begin(); it != intersections.end(); ++it) {
			points.push_back((*it));
			pointsinv.push_back((*it));
		}
		points.push_back(right_int);
		pointsinv.push_back(right_int);

		// Publish the lines themselves
		humanoid_league_msgs::LineInformationInImage field_borders;
		for (auto it = points.begin(); it != points.end() - 1; ++it) {
			geometry_msgs::Point p1;
			p1.x = it->x;
			p1.y = it->y;
			p1.z = 0;

			geometry_msgs::Point p2;
			p1.x = it->x;
			p1.y = it->y;
			p1.z = 0;

			humanoid_league_msgs::LineSegmentInImage seg;
			seg.start = p1;
			seg.end = p2;
			field_borders.segments.push_back(seg);
		}
		field_border.publish(field_borders);

		// Publish the ROI
		Point2f bottomleft, bottomright, topleft, topright;
		bottomleft.x = 0;
		bottomleft.y = 0;
		bottomright.x = img->image.size().width;
		bottomright.y = 0;
		topleft.x = 0;
		topleft.y = img->image.size().height;
		topright.x = img->image.size().width;
		topright.y = img->image.size().height;

		points.push_back(topright);
		points.push_back(topleft);

		pointsinv.push_back(bottomright);
		pointsinv.push_back(bottomleft);

		// Publish ROI
		object_recognition::ROI roi;
		roi.name = "Field ROI";

		for(auto it = points.begin(); it != points.end(); ++it) {
			geometry_msgs::Point p;
			p.x = it->x;
			p.y = it->y;
			roi.points.push_back(p);
		}
		field_roi.publish(roi);

		// Publish filtered area
		vector<Point> pointinv2;
		for(auto it = pointsinv.begin(); it != pointsinv.end(); ++it) {
			pointinv2.push_back((Point) (*it));
		}

		Point* pointinvdata = pointinv2.data();
		const Point* ppt[1] = { pointinvdata };
		int npt[] = { (int) points.size() };

//		fillPoly( field_area_mat, ppt, npt, 1, Scalar( 0, 0, 0 ), 8 );
		std_msgs::Header header;
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", field_area_mat).toImageMsg();
		field_area_img.publish(msg);
		
		//publish field boundary line
//		object_recognition::FieldBoundary msg_line;
//		msg_line = PopulateFieldBmsg(peaks);
//		field_boundary.publish(msg_line);
	}
	else {
		field_area_img.publish(msg);
	}

	// Save information
	saveImage(*nh, field_area_mat, "field", "test", ++image_count);
	saveImage(*nh, img->image, "field", "orig", image_count);
}

void callback_colorspace(const image_acquisition::SoccerColorSpaceConstPtr& msg)
{
	//update colorspace
	lower = Scalar(msg->grass.lower_hue, msg->grass.lower_sat, msg->grass.lower_val);
	upper = Scalar(msg->grass.upper_hue, msg->grass.upper_sat, msg->grass.upper_val);

	ROS_INFO("Updated Colorspace %d %d %d %d %d %d", msg->grass.lower_hue, msg->grass.upper_hue, msg->grass.lower_sat, msg->grass.upper_sat, msg->grass.lower_val, msg->grass.upper_val);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "field_ROI");
	ros::NodeHandle n;
	nh = &n;

    image_transport::ImageTransport it(n);
    image_transport::Subscriber hsv_img = it.subscribe("/camera_input/image_hsv", 1, find_field_area);
    ros::Subscriber colorspace = n.subscribe("/image_acquisition/colorspace", 1, callback_colorspace);
    field_roi = n.advertise<object_recognition::ROI>("/object_recognition/field_ROI", 1);
//    field_boundary = n.advertise<object_recognition::FieldBoundary>("/object_recognition/field_boundary", 1);
    field_area_img = it.advertise("/object_recognition/field_area", 1);
    field_border = n.advertise<humanoid_league_msgs::LineInformationInImage>("/object_recognition/field_borders", 1);

	ros::spin();
}
