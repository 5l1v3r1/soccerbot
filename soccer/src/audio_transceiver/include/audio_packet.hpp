#ifndef AUDIO_PACKET_HPP
#define AUDIO_PACKET_HPP

#include <iostream>

using namespace std;

typedef enum destination_type {
    broadcast,
    unicast,
} DestinationType;

class AudioPacket {
public:
    string message;
    DestinationType destination;
    int frequency;
};


#endif /* AUDIO_PACKET_HPP */

