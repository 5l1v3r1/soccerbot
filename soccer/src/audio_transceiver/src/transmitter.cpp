#include "transmitter.h"

// The constructor should be used to generate an audio packet depending on certain action
// This needs to be called once the robots are all set on their respective positions. 
// All 5 robots would be transmitters. 
Transmitter::Transmitter() {
    Pa_Initialize();  
    if( err != paNoError ){
        std::cout <<"Could not initialize the Portaudio Object in transmitter"<< std::endl; 
        this->~Transmitter( );
    }
    outputParameters.channelCount = 2; /* stereo input */
    outputParameters.sampleFormat = PA_SAMPLE_TYPE;
    outputParameters.suggestedLatency = Pa_GetDeviceInfo(outputParameters.device)->defaultLowInputLatency;
    outputParameters.hostApiSpecificStreamInfo = NULL;
    // Set a PaDeviceIndex using hardware or we will ask for user input. 
}

// This would be converted to a big switch case statement. 
// All communication can occur using this then.
// change command to a template.
// This should not exist actually 
// We should do this in the get message command
Transmitter::Transmitter(std::string command, DestinationType destCommand){ 
    outputParameters.channelCount = 2; /* stereo input */
    outputParameters.sampleFormat = PA_SAMPLE_TYPE;
    outputParameters.suggestedLatency = Pa_GetDeviceInfo(outputParameters.device)->defaultLowInputLatency;
    outputParameters.hostApiSpecificStreamInfo = NULL;
    
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
    
    
    err = Pa_OpenStream(&stream,
                        NULL, // no input 
                        &outputParameters,
                        SAMPLE_RATE,
                        FRAMES_PER_BUFFER,
                        paClipOff,      
                        paTestCallBack,
                        &(newPacket.get_data()));
    
}

int Transmitter::paTestCallBack( const void *inputBuffer, void *outputBuffer,
                                        unsigned long framesPerBuffer,
                                        const PaStreamCallbackTimeInfo* timeInfo,
                                        PaStreamCallbackFlags statusFlags,
                                        void *userData ){
    /* Cast data passed through stream to our structure. */
    RawAudio *data = (RawAudio*)userData;
    float *out = (float*)outputBuffer;
    unsigned int i;
    (void) inputBuffer; /* Prevent unused variable warning. */
    for( i=0; i<framesPerBuffer; i++ ){
        *out++ = data->left_phase;  /* left */
        *out++ = data->right_phase;  /* right */
        /* Generate simple sawtooth phaser that ranges between -1.0 and 1.0. */
        data->left_phase += 0.01f;
        /* When signal reaches top, drop back down. */
        if( data->left_phase >= 1.0f ) data->left_phase -= 2.0f;
        /* higher pitch so we can distinguish left and right. */
        data->right_phase += 0.03f;
        if( data->right_phase >= 1.0f ) data->right_phase -= 2.0f;
    }
    return 0;
}

// This would be controlled by the hardware at all times. As soon as the hardware
// crashes the transmitter should get deleted. 
Transmitter::~Transmitter(){ 
    cout <<"Deleting transmitter object"; 
    Pa_Terminate();
  // clear out all the messages by the machine if broken. 
  // Delete the node now. 
}