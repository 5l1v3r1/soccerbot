#ifndef RECEIVER_H
#define RECEIVER_H

#include <queue>
#include "ros/ros.h"
#include "../../audio/include/audio_packet.hpp"
#include "../../audio/include/portaudio.h"

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

