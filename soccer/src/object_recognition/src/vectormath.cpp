#include "../include/vectormath.hpp"
#include <vectormath.hpp>
#include <opencv2/core.hpp>
#include <unordered_map>
#include <math.h>
#include <humanoid_league_msgs/LineInformationInImage.h>

// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).

Point2f intersection(Vec2f a, Vec2f b) {
	float rho1 = a[0];
	float phi1 = a[1];
	float rho2 = b[0];
	float phi2 = b[1];

	float x = (rho1 / cos(PI / 2 - phi1) - rho2 / cos(PI / 2 - phi2))
			/ (tan(phi2 - PI / 2) - tan(phi1 - PI / 2));

	float y = rho1 / cos(PI / 2 - phi1) + tan(phi1 - PI / 2) * x;

	Point2f intersect(x, y);
	return intersect;
}

Point2f leftScreenIntersection(Vec2f vec, Size2f imgSize) {
	float rho = vec[0];
	float theta = PI / 2 - vec[1];

	float x = imgSize.height;
	float y = imgSize.width;

	float a = 0;
	float b = 0;

	if ((rho / cos(theta)) < x && (rho / cos(theta)) > 0) {
		a = rho / cos(theta);
		b = 0;
//		ROS_ERROR("CASE 1");
	} else if (tan(PI - theta) * (y - rho * sin(theta)) < x && theta < PI / 2) {
		a = x;
		b = rho * sin(theta) - tan(PI / 2 - theta) * (x - rho * cos(theta));
//		ROS_ERROR("CASE 2");
	} else if ((rho * cos(PI - theta) + x) * tan( PI / 2 - (PI - theta))
			+ rho * sin(PI - theta) < y && theta > PI / 2) {
		a = x;
		b = (rho * cos(PI - theta) + x) * tan( PI / 2 - (PI - theta))
				+ rho * sin(PI - theta);
//		ROS_ERROR("CASE 3");
	} else {
		a = tan(PI - theta) * (y - rho * sin(theta));
		b = y;
//		ROS_ERROR("CASE 4");
	}

	Point2f intersect(b, a);
	return intersect;
}

Point2f rightScreenIntersection(Vec2f vec, Size2f imgSize) {
	float rho = vec[0];
	float theta = vec[1];

	float x = imgSize.width;
	float y = imgSize.height;

	float a = 0;
	float b = 0;

	if ((rho / cos(theta)) < x && (rho / cos(theta)) > 0) {
		a = rho / cos(theta);
		b = 0;
//		ROS_ERROR("CASE 1");
	} else if (tan(PI - theta) * (y - rho * sin(theta)) < x && theta < PI / 2) {
		a = x;
		b = rho * sin(theta) - tan(PI / 2 - theta) * (x - rho * cos(theta));
//		ROS_ERROR("CASE 2");
	} else if ((rho * cos(PI - theta) + x) * tan( PI / 2 - (PI - theta))
			+ rho * sin(PI - theta) < y && theta > PI / 2) {
		a = x;
		b = (rho * cos(PI - theta) + x) * tan( PI / 2 - (PI - theta))
				+ rho * sin(PI - theta);
//		ROS_ERROR("CASE 3");
	} else {
		a = tan(PI - theta) * (y - rho * sin(theta));
		b = y;
//		ROS_ERROR("CASE 4");
	}

	Point2f intersect(a, b);
	return intersect;
}

bool isVerticalLine(Vec2f vec, Size2f imgSize) {
	float rho = vec[0];
	float theta = vec[1];

	float x = imgSize.width;
	float y = imgSize.height;

	float rho2 = vec[0];
	float theta2 = PI / 2 - vec[1];

	float x2 = imgSize.height;
	float y2 = imgSize.width;

	if ((rho / cos(theta)) < x && (rho / cos(theta)) > 0) {
		return true; // TODO allow robot to see outside of the field
		if((tan(PI - theta2) * (y2 - rho2 * sin(theta2)) < x2 && theta2 < PI / 2)) {
			return true;
		}
		if(((rho2 * cos(PI - theta2) + x2) * tan( PI / 2 - (PI - theta2)) + rho2 * sin(PI - theta2) < y2 && theta2 > PI / 2)) {
			return true;
		}
	}

	return false;
}

bool sortbyangle(Vec2f a, Vec2f b) {
	return a[1] < b[1];
}

bool sortbydistance(Vec2f a, Vec2f b) {
	return a[0] < b[0];
}

vector<Vec2f> filterRepeats(vector<Vec2f>& lines) {
	KDE kde;
	kde.set_kernel_type(1);
	kde.set_bandwidth_opt_type(1);

	for (auto it = lines.begin(); it != lines.end(); ++it) {
		vector<double> angles;
		angles.push_back(it->val[1]);
		kde.add_data(angles);
	}

	vector<double> pdf;

	double min_x = kde.get_min(0);
	double max_x = kde.get_max(0);
	double x_increment = (max_x - min_x) / 1000.0;

	cout << "# bandwidth var 1: " << kde.get_bandwidth(0) << endl;
	vector<Vec2f> peaks;
	double currentpeak = 0;
	bool up = true;
	for (double x = min_x; x <= max_x; x += x_increment) {
		float p = kde.pdf(x);
		if (p < currentpeak && up == true) {
			float rho = 0;
			float mindist = 10000000;
			for (auto it = lines.begin(); it != lines.end(); ++it) {
				float closeness = abs(it->val[1] - x);
				if (closeness < mindist) {
					mindist = closeness;
					rho = it->val[0];
				}
			}
			Vec2f vec(rho, x);
			peaks.push_back(vec);

			up = false;
		}
		if (p > currentpeak)
			up = true;
		currentpeak = p;
	}

	return peaks;
}

// TODO Better filter for field lines
vector<Vec2f> filterUnparallelRepeats(vector<Vec2f>& lines) {
	if(lines.size() <= 1) return lines;

	sort(lines.begin(), lines.end(), sortbyangle);
	vector<Vec2f> filteredLines;

	for (auto it = lines.begin(); it != lines.end() - 1; ++it) {
		float anglediff = abs((*it)[1] - (*(it+1))[1]);
		float rhodiff = abs((*it)[0] - (*(it+1))[0]);

		if(anglediff > ANGLE_PROXIMITY_MIN || anglediff < ANGLE_PROXIMITY_MAX) {
			if(rhodiff > RHO_PROXIMITY_MIN) {
				filteredLines.push_back((*it));
			}
		}
	}

	return filteredLines;
}


vector<Vec2f> filterByAngle(vector<Vec2f>& lines, float angleStart, float angleEnd) {
	vector<Vec2f> filtered_lines;
	for(auto it = lines.begin(); it != lines.end(); ++it) {
		if((*it)[1] > angleStart && (*it)[1] < angleEnd) {
			filtered_lines.push_back((*it));
		}
	}
	return filtered_lines;
}

void drawLinesOnImg(Mat& img, vector<Vec2f>& lines, Scalar color) {
	for (size_t i = 0; i < lines.size(); i++) {
		float rho = lines[i][0], theta = lines[i][1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a * rho, y0 = b * rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		line(img, pt1, pt2, color, 1, CV_AA);
	}
}

void drawLinesOnImgCartesian(Mat& img, humanoid_league_msgs::LineInformationInImage& lines_cart, Scalar color) {
	for(auto it = lines_cart.segments.begin(); it != lines_cart.segments.end(); ++it){
		Point pt1,pt2;
		pt1.x = it->start.x;
		pt1.y = it->start.y;
		pt2.x = it->end.x;
		pt2.y = it->end.y;
		line(img, pt1, pt2, color, 1, CV_AA);
	}
}

void saveImage(ros::NodeHandle& nh, const Mat& img, string folder, string name, int count) {
	bool image_test;
	nh.getParam("image_test", image_test);
	if (image_test) {
		std::stringstream ss;
		ss << "../../../src/object_recognition/test/" << folder << "/" << name << count << ".png";
		std::string fileName = ss.str();

		try {
			imwrite(fileName, img);
		} catch (runtime_error& ex) {
			ROS_ERROR(ex.what());
		}
	}
}
