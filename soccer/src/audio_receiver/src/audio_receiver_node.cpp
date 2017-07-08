#include "ros/ros.h"
#include "std_msgs/String.h"
#include "transmitter.h"
#include "reciever.h"
#include "audio_packet.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "audio_transciever");
    ros::NodeHandle n;

    Transmitter t();
    Reciever r();
    
    AudioPacket p;
    p.message = "Hello World";
    
    t.send_message(p);
    
    ros::spin();
    return EXIT_SUCCESS;
}