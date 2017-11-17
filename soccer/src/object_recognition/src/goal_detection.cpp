#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <humanoid_league_msgs/GoalPartsInImage.h>
#include <humanoid_league_msgs/GoalInImage.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace ros;

Publisher goal_in_image;
Subscriber goal_part_candidates;

void goal_detection(const humanoid_league_msgs::GoalPartsInImagePtr& msg) {


}

int main(int argc, char **argv) {

    init(argc, argv, "goal_detection");
    NodeHandle n;

    goal_part_candidates = n.subscribe("/object_recognition/goal_part_candidates", 1, &goal_detection);
    goal_in_image = n.advertise<humanoid_league_msgs::GoalInImage>("/object_recognition/goal_in_image", 1);

    ros::spin();
}
