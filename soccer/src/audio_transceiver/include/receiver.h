#ifndef RECEIVER_H
#define RECEIVER_H

#include <queue>
#include "audio_packet.hpp"
#include "portaudio.h"

using namespace std;

class Receiver {
public:
    Receiver();
    Receiver(const Receiver& orig);
    virtual ~Receiver();
    
    void record_playback();
private:
    PaStreamParameters inputParameters,outputParameters;
    PaStream* stream;
    
    std::queue<AudioPacket> audio_queue;
};

#endif /* RECIEVER_H */

