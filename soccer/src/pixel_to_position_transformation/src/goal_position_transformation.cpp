#include <ros/ros.h>
#include <iostream>
#include "stdio.h"
#include <visualization_msgs/Marker.h>
#include <humanoid_league_msgs/GoalInImage.h>
#include <humanoid_league_msgs/GoalRelative.h>
#include <humanoid_league_msgs/LineInformationInImage.h>
#include <humanoid_league_msgs/LineInformationRelative.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "../include/point_transform.hpp"

#include <image_transport/image_transport.h>   //testing
#include <opencv2/calib3d/calib3d.hpp>
#include <std_msgs/Float32.h>


using namespace std;
using namespace ros;
using namespace cv;

ros::NodeHandle* nh;

Subscriber obstacle_in_image;
Subscriber line_in_image;
Subscriber d_subscribe_;
image_transport::Subscriber rawimg;
image_transport::Publisher inter_img;  //testing
vector<geometry_msgs::Point> inters;   //testing
bool inters_ready = false;  		   //testing
Publisher goal_relative;
Publisher post_lines_gui;
humanoid_league_msgs::LineInformationInImage field_line;
bool field_line_ready = false;
const double tolerance = 0;

Point2f intersection_cartesian(Point2f start1, Point2f end1, Point2f start2, Point2f end2){
	float a1 = 0;
	float a2 = 0;
	float b1 = 0;
	float b2 = 0;

	Point2f rtn_p = { 0 };

	if( start1.x == end1.x &&
		start2.x != end2.x	)
	{
		rtn_p.x = start1.x;
		a2 = (end2.y-start2.y) / (end2.x - start2.x);
		b2 = end2.y - a2 * end2.x;
		rtn_p.y  = a2 * rtn_p.x + b2;

	}
	else if( start2.x == end2.x)
	{
		rtn_p.x = start2.x;
		a1 = (end1.y-start1.y) / (end1.x - start1.x);
		b1 = end1.y - a2 * end1.x;
		rtn_p.y  = a1 * rtn_p.x + b1;
	}
	else
	{
		a1 = (end1.y-start1.y) / (end1.x - start1.x);
		a2 = (end2.y-start2.y) / (end2.x - start2.x);
		b1 = end1.y - a1 * end1.x;
		b2 = end2.y - a2 * end2.x;

		if( a1 != a2 )
		{
			rtn_p.x = (b1-b2)/(a2-a1);
			rtn_p.y = (a2*b1 -  a1*b2)/(a2-a1);
		}
	}

	//parallel

	return rtn_p;

}

static void find_obstacle_position(const humanoid_league_msgs::LineInformationInImagePtr& msg) {
	/*geometry_msgs::Point p1 = msg->left_post.foot_point;
	geometry_msgs::Point p2 = msg->right_post.foot_point;
	geometry_msgs::Point p1_3d = point2d_to_3d(p1, camera_size.height, camera_size.width);
	geometry_msgs::Point p2_3d = point2d_to_3d(p2, camera_size.height, camera_size.width);

	humanoid_league_msgs::GoalRelative output;
	output.left_post = p1_3d;
	output.right_post = p2_3d;*/

	if( field_line_ready )
	{

		//find intersection
		vector<geometry_msgs::Point> pseudo_inters;
		for( int i = 0; i < field_line.segments.size();i++) {
			if( (field_line.segments[i].end.y - field_line.segments[i].start.y) < tolerance){
				for(int j = 0; j < msg->segments.size();j++){
					geometry_msgs::Point p;
					Point2f tmp_p,start1,start2,end1,end2;
					start1.x = field_line.segments[i].start.x;
					start1.y = field_line.segments[i].start.y;
					end1.x = field_line.segments[i].end.x;
					end1.y = field_line.segments[i].end.y;
					start2.x = msg->segments[j].start.x;
					start2.y = msg->segments[j].start.y;
					end2.x = msg->segments[j].end.x;
					end2.y = msg->segments[j].end.y;

					tmp_p = intersection_cartesian(start1, end1, start2, end2 );
					p.x = tmp_p.x;
					p.y = tmp_p.y;

					pseudo_inters.push_back(p);
				}

			}
		}

		//2d->3d
		humanoid_league_msgs::LineInformationRelative output;
		for( int i = 0;i < pseudo_inters.size();i++)
		{
			geometry_msgs::Point p1_3d = point2d_to_3d(pseudo_inters[i], camera_size.height, camera_size.width);
			geometry_msgs::Point p2_3d = point2d_to_3d(pseudo_inters[i], camera_size.height, camera_size.width);
			p2_3d.z += 10;
			//ROS_INFO("p1_x: %f, p1_y: %f", p1_3d.x,p1_3d.y);

			humanoid_league_msgs::LineSegmentRelative segment;
			segment.start = p1_3d;
			segment.end = p2_3d;
			output.segments.push_back(segment);

		}

		draw_lines2(post_lines_gui, output, 10);  //blue version
		goal_relative.publish(output);

		//testing
		inters_ready = true;
		inters.clear();
		for(int i = 0; i < pseudo_inters.size();i++)
			inters.push_back(pseudo_inters[i]);

	}

}

static void update_lines_in_image(const humanoid_league_msgs::LineInformationInImagePtr& msg) {
	field_line = *msg;
	field_line_ready = true;
}

static void update_img(const sensor_msgs::ImageConstPtr& msg){    //testing
	cv_bridge::CvImageConstPtr img;
	try {
		img = cv_bridge::toCvShare(msg, "");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	if(inters_ready)
	{
		Mat tmp_img = img->image.clone();
		for(int i =0;i<inters.size();i++){
			Point tmp_point;
			tmp_point.x = (int)inters[i].x;
			tmp_point.y = (int)inters[i].y;
			circle(tmp_img, tmp_point, 2, Scalar(0,0,255));
		}

		std_msgs::Header header;
		sensor_msgs::ImagePtr inter_img_msg = cv_bridge::CvImage(header, "bgr8", tmp_img).toImageMsg();
		inter_img.publish(inter_img_msg);

	}

	camera_size = img->image.size();
}

static void update_d_(const std_msgs::Float32::ConstPtr& msg) {
	D = msg->data;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "goal_position_transformation");
    ros::NodeHandle n;
    nh = &n;  //testing

    image_transport::ImageTransport it(n);
    rawimg = it.subscribe("/camera_input/image_raw", 1, &update_img);
    inter_img = it.advertise("/object_recognition/img_intersection", 1);  //testing

    obstacle_in_image = n.subscribe("/object_recognition/post_in_image", 1, &find_obstacle_position);
    line_in_image = n.subscribe("/object_recognition/lines_in_image", 1, update_lines_in_image);
    //goal_relative = n.advertise<humanoid_league_msgs::GoalRelative>("pixel_to_position_transformation/goal_relative", 1);
    goal_relative = n.advertise<humanoid_league_msgs::LineInformationRelative>("pixel_to_position_transformation/goal_relative", 1);
    post_lines_gui = n.advertise<visualization_msgs::Marker>("/pixel_to_position_transformation/post_lines", 1);
    d_subscribe_ = n.subscribe("/pixel_to_position_transformation/D", 1, update_d_);

    ros::spin();
}
