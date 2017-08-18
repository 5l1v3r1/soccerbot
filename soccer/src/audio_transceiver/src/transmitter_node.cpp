#include "ros/ros.h"
#include "std_msgs/String.h"
#include "transmitter.h"
#include "receiver.h"
#include "audio_packet.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "audio_transceiver");
    ros::NodeHandle n;

    Transmitter t;
    //std::cout<<"CHECK DEBUG\n"; 
    //AudioPacket p;
    //p.set_message("Hello World");
    
    //t.send_message(p);
    
    //ros::spin();
    return EXIT_SUCCESS;
}