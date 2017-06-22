#ifndef AUDIO_PACKET_HPP
#define AUDIO_PACKET_HPP

#include <iostream>

using namespace std;

enum destination_type {
    broadcast,
    unicast,
};

typedef class AudioPacket {
    string message;
    destination_type destination = destination_type.broadcast;
    int frequency = 1000;
};


#endif /* AUDIO_PACKET_HPP */

