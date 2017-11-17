#include "ros/ros.h"
#include "std_msgs/String.h"

#include "../../audio/include/audio_packet.hpp"
#include "../../audio/include/receiver.h"
#include "../../audio/include/transmitter.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "audio_transceiver");
    ros::NodeHandle n;

    Transmitter t;
    
    DestinationType destCommand = broadcast;
    t.generateAudioPacket("Hello World", destCommand);
    
    //std::cout<<"CHECK DEBUG\n"; 
    //AudioPacket p;
    //p.set_message("Hello World");
    
    //t.send_message(p);
    
    //ros::spin();
    return EXIT_SUCCESS;
}