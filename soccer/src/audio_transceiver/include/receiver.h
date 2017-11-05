#ifndef RECEIVER_H
#define RECEIVER_H

#include <queue>
#include "audio_packet.hpp"
#include "portaudio.h"
#include "ros/ros.h"

using namespace std;

class Receiver {
public:
    Receiver();
    Receiver(const Receiver& orig);
    virtual ~Receiver();
    
    void record_playback();
private:
    PaStreamParameters inputParameters;
    PaStream* stream;
    std::queue<AudioPacket> received_queue;
};

#endif /* RECIEVER_H */

