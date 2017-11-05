#include "ros/ros.h"
#include "std_msgs/String.h"
#include "transmitter.h"
#include "receiver.h"
#include "audio_packet.hpp"

/*int main(int argc, char **argv) {
    ros::init(argc, argv, "audio_transceiver");
    ros::NodeHandle n;

    Receiver r;
    
    //t.send_message(p);
    
    ros::spin();
    return EXIT_SUCCESS;
}*/