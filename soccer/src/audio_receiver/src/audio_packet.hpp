#ifndef AUDIO_PACKET_HPP
#define AUDIO_PACKET_HPP

#include <iostream>

using namespace std;

enum destination_type {
    broadcast,
    unicast,
};

typedef class audio_packet {
    string message;
    destination_type destination = destination_type.broadcast;
};


#endif /* AUDIO_PACKET_HPP */

