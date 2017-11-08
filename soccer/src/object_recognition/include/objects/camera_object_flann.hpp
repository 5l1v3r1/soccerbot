#ifndef CAMERA_OBJECT_FLANN_HPP
#define CAMERA_OBJECT_FLANN_HPP

#include <std_msgs/Header.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "camera_object.hpp"
#include "../pose_estimation/Model.h"
#include "../pose_estimation/RobustMatcher.h"
#include "../pose_estimation/Mesh.h"
#include "../../../audio/include/constants.hpp"


using namespace std;
using namespace cv;

class CameraObjectFlann : public CameraObject {
public :
    CameraObjectFlann();
    
private :
    Model model;
    Mesh mesh; // The mesh shape of the object
    vector<Point3f> modelPoints; // The 3D points on the model
    
    RobustMatcher rmatcher;
    Ptr<FeatureDetector> orb;
    Ptr<flann::IndexParams> indexParams;
    Ptr<flann::SearchParams> searchParams;
    Ptr<DescriptorMatcher> matcher;
    
    vector<DMatch> good_matches; // 3D Points of the model
    vector<KeyPoint> keypoints_scene;
    
    
    
};

#endif /* CAMERA_OBJECT_FLANN_HPP */

