#ifndef AUDIO_PACKET_HPP
#define AUDIO_PACKET_HPP

#include <iostream>
#include "constants.hpp"
#include "portaudio.h"

using namespace std;

typedef enum destination_type {
    broadcast,
    unicast,
} DestinationType;

typedef struct raw_audio{
    int frameIndex; /* Index into sample array. */
    int maxFrameIndex;
    SAMPLE *recordedSamples;
} RawAudio;

class AudioPacket {
private:
    int numBytes;
    int totalFrames;
public:
    AudioPacket();
    
    static int numSamples;
    
    string message;
    DestinationType destination;
    RawAudio data;
    
    int frequency;
    
    void convert_message_to_raw_audio();
};


#endif /* AUDIO_PACKET_HPP */

