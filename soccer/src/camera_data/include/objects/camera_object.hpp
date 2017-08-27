#ifndef CAMERA_TRANSFORMER_HPP
#define CAMERA_TRANSFORMER_HPP

#include "constants.hpp"
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class CameraTransformer {
public :
    static int robotHeight;
    static int cameraDistance;
    
    
    geometry_msgs::PoseWithCovarianceStamped cameraToRobotPosition(cv::Point position);
    
private :
    
    
    
};

#endif /* CAMERA_TRANSFORMER_HPP */

