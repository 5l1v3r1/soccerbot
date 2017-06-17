#include "ball.hpp"
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv) {
    // Set up ROS.
    ros::init(argc, argv, "soccer_ball");
    ros::NodeHandle n;

    Ball ball(n);
    ball.draw();

    ros::spin();
}