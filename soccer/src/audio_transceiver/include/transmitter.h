#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include <audio_packet.hpp>
#include "portaudio.h"

class Transmitter {
public:
    bool send_message(AudioPacket& packet);
    
    Transmitter();
    Transmitter(string command, DestinationType destCommand);
    Transmitter(const Transmitter& orig);
    bool generateAudioPacket(string command, DestinationType destCommand); 
    virtual ~Transmitter();
private:
    PaStreamParameters outputParameters;
    // This is just here if we have multiple commands going in faster 
    // than the processing speed of the robot. 
    //std::queue<AudioPacket> transmit_queue;
};

#endif /* TRANSMITTER_H */

