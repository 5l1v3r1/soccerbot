#ifndef AUDIO_PACKET_HPP
#define AUDIO_PACKET_HPP

// 40 STEPS/SECOND OF AUDIO DATA  
//#define SAMPLE_RATE 48000

#include <iostream>
#include "constants.hpp"
#include "portaudio.h"
#include <math.h>

using namespace std;


// This might need a number of robots and a much further instruction. 
// eg. a unicast means one robot now that one robot needs to be defined here. 

typedef enum destination_type {
    broadcast,
    unicast,
} DestinationType;



class AudioPacket {
private:
     
    //int priority;
    // CONFUSION: 
    // --------------------------------------------------------------------------
    // Samples per frame = (sample rate)/FPS
    // CHECK: 
    // I think we would much rather store the sample rate and frame rate than num of samples 
    // -----------------------------------------------------------------------------------
    // I am not sure if there is a connection between the number of Bytes and number of frames
    int numBytes;
    //Related to the frame rate of the data. Again the theory not good enough for this 
    // type of communication. Frames accumulated must be equal to the num of samples thus
    // does not really make sense to actually call this function. 
    int totalFrames;
    // Have to understand how to connect the number of samples to the sampling rate. 
    // Sampling rate can be different for different devices. 
    // Probably make a list from where we can choose which one to take something like enum.
    
    // Every 100ms we would be trying to figure out the 
    //float tone_buffer[9600][5][5];

    float lower_freq = 50; 
    float higher_freq = 500; 
    const int numSamples = 100;
    
    // Will be used to convert message to rawAudio data. 
    string message;
    
    // I think this needs to be an array of frequency for every message rather than
    // a single frequency. But let's just have a one frequency message for now. 
    int frequency;
    
    RawAudio data;
    PaError Err; 
    // The destination would store all the information about the receiver. 
    DestinationType destination;
public:
    // The constructor function has been kept for security purposes. Could be found useful 
    // maybe to generate random noise for no reason with invalid tokens. This might help
    // us in the future. For now this would be empty but later on this would have various
    // different frequency noises. 
    AudioPacket();
    
    // AudioPacket generation will be done here. We would be adding each packet in a queue.
    // Each Packet should be given a priority as well.So that it could be used to 
    // align in the priority queue. 
    AudioPacket(string command, DestinationType destCommand);
    
    // This needs extra security. This needs to be authenticated. 
    const string& get_message(); 
    void set_message(const string& _message); 
    
    RawAudio& get_data(); 
    void set_data(const RawAudio& _message); 
    // Let's just use it without any priority.
    // -------------------------------------------------------------------------
    //void get_priority(); 
    //update_priority will be called every time we would be able to send a message successfully. 
    // Will be called by update priority queue. 
    // should be updated in audio_packet_queue
    //void update_priority(); 
    //--------------------------------------------------------------------------

    // Take the message to convert to raw audio file.
    // Not sure if this would be required.
    //void convert_message_to_raw_audio();
};


#endif /* AUDIO_PACKET_HPP */

