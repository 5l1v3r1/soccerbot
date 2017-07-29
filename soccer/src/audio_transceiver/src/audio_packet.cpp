#include "audio_packet.hpp"

int AudioPacket::numSamples = 0;

AudioPacket::AudioPacket() {
    
}

AudioPacket::AudioPacket(string command, DestinationType destCommand){ 
    // check for security here. Who is sending the command 
    message = command; 
}


const string& AudioPacket::get_message(){ 
    return message;    
}


void AudioPacket::set_message(const string& _message){ 
    message = _message; 
} 

void get_Priority(){ 
    

} 

void update_priority(){ 


}

