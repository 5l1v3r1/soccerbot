#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <humanoid_league_msgs/LineInformationInImage.h>
#include "vectormath.hpp"

using namespace std;
using namespace ros;
using namespace cv;

ros::NodeHandle* nh;

Publisher line_points_in_image;
Publisher lines_in_image;
image_transport::Subscriber field_img;
image_transport::Publisher line_img;
Subscriber field_border;
int image_count = 0;

Scalar lower = Scalar(0, 0, 165);
Scalar upper = Scalar(255, 105, 255);

void detect_lines(const sensor_msgs::ImageConstPtr& msg) {
	ROS_INFO("Line Area");

	cv_bridge::CvImageConstPtr img;
	try {
		img = cv_bridge::toCvShare(msg, "");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	Mat mask, mask2, final;
	vector<Vec2f> lines, fieldlines;

	bilateralFilter(img->image, mask, 9, 75, 75);
	Canny(mask, mask2, 70, 140, 3);
	HoughLines(mask2, lines, 1, CV_PI / 180, 120, 0, 0);
	fieldlines = filterUnparallelRepeats(lines);

	final = img->image.clone();
	drawLinesOnImg(final, fieldlines, Scalar(255,0,0));

	// Extract the intersections of the line
	vector<Point2f> intersections;
	for(int i = 0; i < fieldlines.size(); ++i) {
		for(int j = 0; j < fieldlines.size(); ++j) {
			if(abs(fieldlines[i][1] - fieldlines[j][1]) < CV_PI / 24) continue;

			if(i == j) continue;
			Point2f intersect = intersection(fieldlines[i], fieldlines[j]);
			intersections.push_back(intersect);
			circle(final, intersect, 5, Scalar(0,255,0));
		}
	}

	// Send off the information
	humanoid_league_msgs::LineInformationInImage lineinfo;

	for(auto it = fieldlines.begin(); it != fieldlines.end(); ++it) {
		Point2f p1 = leftScreenIntersection(*it, img->image.size());
		Point2f p2 = rightScreenIntersection(*it, img->image.size());
		circle(final, p1, 5, Scalar(0,255,0));
		circle(final, p2, 5, Scalar(0,255,0));
		humanoid_league_msgs::LineSegmentInImage seg;
		seg.start.x = p1.x;
		seg.start.y = p1.y;
		seg.end.x = p2.x;
		seg.end.y = p2.y;

		lineinfo.segments.push_back(seg);
	}

	lines_in_image.publish(lineinfo);

	std_msgs::Header header;
	sensor_msgs::ImagePtr line_img_msg = cv_bridge::CvImage(header, "bgr8", final).toImageMsg();
	line_img.publish(line_img_msg);

	saveImage(*nh, final, "lines", "test", ++image_count);
	saveImage(*nh, img->image, "lines", "orig", image_count);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "line_detection");
	ros::NodeHandle n;
	nh = &n;

	image_transport::ImageTransport it(n);
	field_img = it.subscribe("/object_recognition/field_area", 1, &detect_lines);

	line_points_in_image = n.advertise<sensor_msgs::PointCloud2>("/object_recognition/line_points_in_image", 1);
	lines_in_image = n.advertise<humanoid_league_msgs::LineInformationInImage>("/object_recognition/lines_in_image", 1);
    line_img = it.advertise("/object_recognition/line_area", 1);

	ros::spin();
}
