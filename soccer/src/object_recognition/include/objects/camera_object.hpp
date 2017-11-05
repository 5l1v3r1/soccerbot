#ifndef CAMERA_TRANSFORMER_HPP
#define CAMERA_TRANSFORMER_HPP

#include "constants.hpp"
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

class CameraObject {
public :
    CameraObject();
    
    vector<Point3f> model_points;
    vector<Point3f> scene_points;
    vector<Point2f> camera_points;
private :
    
};

#endif /* CAMERA_TRANSFORMER_HPP */

