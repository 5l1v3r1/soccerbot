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

};

#endif /* TRANSMITTER_H */

