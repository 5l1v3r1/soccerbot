#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include <audio_packet.hpp>
#include "portaudio.h"
#include <string.h>

class Transmitter {
public:
    bool send_message(AudioPacket& packet);
    static int paTestCallBack(  const void *inputBuffer, 
                                void *outputBuffer,
                                unsigned long framesPerBuffer,
                                const PaStreamCallbackTimeInfo* timeInfo,
                                PaStreamCallbackFlags statusFlags,
                                void *userData );
    Transmitter();
    Transmitter(string command, DestinationType destCommand);
    Transmitter(const Transmitter& orig);
    bool generateAudioPacket(string command, DestinationType destCommand); 
    static void StreamFinished( void* userData );
    virtual ~Transmitter();
private:
    PaStreamParameters outputParameters;
    PaError err;
    PaStream *stream;
    
    // This is just here if we have multiple commands going in faster 
    // than the processing speed of the robot. 
    //std::queue<AudioPacket> transmit_queue;
};

#endif /* TRANSMITTER_H */

