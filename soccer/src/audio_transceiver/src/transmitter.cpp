#include "transmitter.h"
#include "portaudio.h"
#include <string.h>
// The constructor should be used to generate an audio packet depending on certain action
Transmitter::Transmitter() {
    
}

// This would be converted to a big switch case statement. 
// All communication can occur using this then.
// change command to a template.
Transmitter::Transmitter(std::string command, DestinationType destCommand){ 
    if (command.compare("P")){ 
        std::string temp = "Pass";
        generateAudioPacket(temp,destCommand); 
    }
    else if(command.compare("S")){ 
        std::string temp = "Shoot";
        generateAudioPacket(temp,destCommand); 
    }
}


Transmitter::Transmitter(const Transmitter& orig) {
}


// This would communicate with the transmitter to send the data packet. 
//Basically this would be doing the talking part of the whole case. 
bool Transmitter::send_message(AudioPacket& packet) {
    
}

bool Transmitter::generateAudioPacket(std::string command, DestinationType destCommand){
    
    // Every message should contain a unique token Number.
    //destCommand contains the information whether it is Unicast or broadcast  
    AudioPacket newPacket(command,destCommand);
    
}

// This would be controlled by the hardware at all times. As soon as the hardware
// crashes the transmitter should get deleted. 
Transmitter::~Transmitter(){ 
  // clear out all the messages by the machine if broken. 
  // Delete the node now. 
}