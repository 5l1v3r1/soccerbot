#include "transmitter.h"

// The constructor should be used to generate an audio packet depending on certain action
// This needs to be called once the robots are all set on their respective positions. 
// All 5 robots would be transmitters. 

Transmitter::Transmitter() {
    //ros::init(argc, argv, "talker");
    //ros::NodeHandle n; }
    Pa_Initialize();
    std::cout << Pa_GetDeviceCount() << std::endl;
    outputParameters.device = Pa_GetDefaultOutputDevice(); /* default output device */

    if (outputParameters.device == paNoDevice) {
        std::cerr << "Error: No default output device.\n";
        this->~Transmitter();
    }
    outputParameters.channelCount = 2; /* stereo input */
    outputParameters.sampleFormat = PA_SAMPLE_TYPE;
    //outputParameters.device = Pa_GetDeviceCount();

    //std::cout << Pa_GetDeviceInfo(outputParameters.device);
    //outputParameters.suggestedLatency = Pa_GetDeviceInfo(outputParameters.device)->defaultLowInputLatency;


    outputParameters.hostApiSpecificStreamInfo = NULL;
    // Set a PaDeviceIndex using hardware or we will ask for user input. 

    
}

// This would be converted to a big switch case statement. 
// All communication can occur using this then.
// change command to a template.
// This should not exist actually 
// We should do this in the get message command

Transmitter::Transmitter(std::string command, DestinationType destCommand) {
    outputParameters.device = Pa_GetDefaultOutputDevice(); /* default output device */
    if (outputParameters.device == paNoDevice) {
        fprintf(stderr, "Error: No default output device.\n");
        this->~Transmitter();
    }
    outputParameters.channelCount = 2; /* stereo input */
    outputParameters.sampleFormat = PA_SAMPLE_TYPE;
    outputParameters.suggestedLatency = Pa_GetDeviceInfo(outputParameters.device)->defaultLowInputLatency;
    outputParameters.hostApiSpecificStreamInfo = NULL;
    // Not the right place to have this.
    if (command.compare("P")) {
        std::string temp = "Pass";
        generateAudioPacket(temp, destCommand);
    } else if (command.compare("S")) {
        std::string temp = "Shoot";
        generateAudioPacket(temp, destCommand);
    }
}

Transmitter::Transmitter(const Transmitter& orig) {
}


// This would communicate with the transmitter to send the data packet. 
//Basically this would be doing the talking part of the whole case. 

bool Transmitter::send_message(AudioPacket& packet) {

}

// TODO Every message should contain a unique token Number.
//destCommand contains the information whether it is Unicast or broadcast  
bool Transmitter::generateAudioPacket(std::string command, DestinationType destCommand) {
    AudioPacket newPacket;
    err = Pa_OpenStream(&stream,
            NULL, // no input 
            &outputParameters,
            SAMPLE_RATE,
            FRAMES_PER_BUFFER,
            paClipOff,
            paTestCallBack,
            &(newPacket.get_data()));

    if (err != paNoError) {
        std::cout << "Error in opening stream" << endl;
        return false;
    }
    
    err = Pa_SetStreamFinishedCallback(stream, &StreamFinished);
    if (err != paNoError) {
        std::cout << "Could not set the final callback\n";
        return false;
    }

    err = Pa_StartStream(stream);
    if (err != paNoError) {
        std::cout << "Could not open stream\n";
        return false;
    }

    Pa_Sleep(1 * 1000); // Sleep 5 
    err = Pa_StopStream(stream);
    if (err != paNoError) {
        std::cout << "Could not stop stream \n";
        return false;
    }
    err = Pa_CloseStream(stream);
    if (err != paNoError) {
        std::cout << "Could not close stream\n";
        return false;
    }
    Pa_Terminate();
    std::cout << "Test finished.\n";

    return true;

}

void Transmitter::StreamFinished(void* userData) {
    RawAudio *data = (RawAudio *) userData;
    std::cout << "Stream Completed.\n";
}

int Transmitter::paTestCallBack(const void *inputBuffer, void *outputBuffer,
        unsigned long framesPerBuffer,
        const PaStreamCallbackTimeInfo* timeInfo,
        PaStreamCallbackFlags statusFlags,
        void *userData) {
       
    // Cast data passed through stream to our structure.
    RawAudio *data = (RawAudio*) userData;
    float *out = (float*) outputBuffer;
    
    float tempOut; 
    //std::array<float, NUMBER_OF_STEPS> tempOut;
    (void) inputBuffer; // Prevent unused variable warning.
    
//    cout << framesPerBuffer << endl;
    
    for (unsigned i = 0; i < framesPerBuffer; i++ ){
//        cout << "Left: "<< data->left_phase;
        tempOut = data->tone_buffer[data->left_phase];
        //for (int i = 0; i < 2; i++){
        *out++ = tempOut;  // left 
        //}
        *out++ = 0 ; 
        *out++ = 0; 
//        cout << "| Right:" << data->right_phase << endl;
        tempOut = data->tone_buffer[data->right_phase];
        for (int i = 1; i < 1; i++){    
        *out++ = tempOut;  // right 
        }

        // Generate simple sawtooth phaser that ranges between -1.0 and 1.0. 

        data->left_phase += 1; // When signal reaches top, drop back down.
        if( data->left_phase >= BUFFER_SIZE )
            data->left_phase -= BUFFER_SIZE;

        data->right_phase += 3; // higher pitch so we can distinguish left and right. 
        if( data->right_phase >= BUFFER_SIZE )
            data->right_phase -= BUFFER_SIZE;
    }

    return paContinue;
}

// This would be controlled by the hardware at all times. As soon as the hardware
// crashes the transmitter should get deleted. 

Transmitter::~Transmitter() {
    cout << "Deleting transmitter object\n";
    Pa_Terminate();
    // clear out all the messages by the machine if broken. 
    // Delete the node now. 
}