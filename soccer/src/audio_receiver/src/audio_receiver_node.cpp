#include "ros/ros.h"
#include "std_msgs/String.h"
#include "transmitter.h"
#include "reciever.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "audio_transciever");
    ros::NodeHandle n;

    Transmitter t();
    Reciever r();

    ros::spin();
    return EXIT_SUCCESS;
}