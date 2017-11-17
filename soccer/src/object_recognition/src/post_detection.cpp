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
image_transport::Subscriber hsv_img;

void findgoal(const sensor_msgs::ImageConstPtr& msg) {


}

int main(int argc, char **argv) {

    init(argc, argv, "goal_detection");
    NodeHandle n;

    image_transport::ImageTransport it(n);
    hsv_img = it.subscribe("/object_recognition/game_ROI", 1, &findgoal);
    goal_in_image = n.advertise<humanoid_league_msgs::GoalPartsInImage>("/object_recognition/goal_part_candidates", 1);

    ros::spin();
}
