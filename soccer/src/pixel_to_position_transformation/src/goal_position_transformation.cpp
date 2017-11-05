#include <ros/ros.h>
#include <iostream>
#include "stdio.h"
using namespace std;

int main(int argc, char **argv) {

    ros::init(argc, argv, "goal_position_transformation");
    ros::NodeHandle n;
    ros::spin();
}
