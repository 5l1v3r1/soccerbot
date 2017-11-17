#include "../../audio/include/receiver.h"

#include <stdio.h>

#include "../../audio/include/portaudio.h"
using namespace std;

Receiver::Receiver() {
    int err = Pa_Initialize();
    if (err != paNoError){
        std::cout<<"Could not initialize the Portaudio Object in the receiver\n";
        this ->~Receiver();
    };

    inputParameters.device = Pa_GetDefaultInputDevice(); /* default input device */
    if (inputParameters.device == paNoDevice) {
        fprintf(stderr, "Error: No default input device.\n");
        this->~Receiver();     
    }
    inputParameters.channelCount = 2; /* stereo input */
    inputParameters.sampleFormat = PA_SAMPLE_TYPE;
    inputParameters.suggestedLatency = Pa_GetDeviceInfo(inputParameters.device)->defaultLowInputLatency;
    inputParameters.hostApiSpecificStreamInfo = NULL;
}


static int recordCallback(  const void *inputBuffer, void *outputBuffer,
                            unsigned long framesPerBuffer,
                            const PaStreamCallbackTimeInfo* timeInfo,
                            PaStreamCallbackFlags statusFlags,
                            void *userData) {
    RawAudio* data = (RawAudio*) userData;
    
    const SAMPLE* rptr = (const SAMPLE*) inputBuffer;
    
    int finished;
    /*
    SAMPLE* wptr = &data->recordedSamples[data->frameIndex * NUM_CHANNELS];
    long framesToCalc;
    long i;
    
    unsigned long framesLeft = data->maxFrameIndex - data->frameIndex;

    (void) outputBuffer; // Prevent unused variable warnings. 
    (void) timeInfo;
    (void) statusFlags;
    (void) userData;

//    if (framesLeft < framesPerBuffer) {
//        framesToCalc = framesLeft;
//        finished = paComplete;
//    } else {
//        framesToCalc = framesPerBuffer;
//        finished = paContinue;
//    }

    if (inputBuffer == NULL) {
        for (i = 0; i < framesToCalc; i++) {
            *wptr++ = SAMPLE_SILENCE; // left 
            if (NUM_CHANNELS == 2) *wptr++ = SAMPLE_SILENCE; // right 
        }
    } else {
        for (i = 0; i < framesToCalc; i++) {
            *wptr++ = *rptr++; //left 
            if (NUM_CHANNELS == 2) *wptr++ = *rptr++; // right
        }
    }
    data->frameIndex += framesToCalc;
    */
    return finished;
}

void Receiver::record_playback() {
    AudioPacket packet;
    // This push would be done in the top class of the receiver.
    //audio_queue.push(packet);
    RawAudio* data = &(packet.get_data());
    
    int err;
    err = Pa_OpenStream(
            &stream,
            &inputParameters,
            NULL, // &outputParameters,
            SAMPLE_RATE,
            FRAMES_PER_BUFFER,
            paClipOff, //we won't output out of range samples so don't bother clipping them 
            recordCallback,
            data);
    if (err != paNoError){ std::cout<<"Could not open the input stream.";}

    err = Pa_StartStream(stream);
    if (err != paNoError) {std::cout<<"Could not start input stream";}
    fflush(stdout);

    while ((err = Pa_IsStreamActive(stream)) == 1) {
        Pa_Sleep(1000);
        //printf("index = %d\n", data->frameIndex);
        fflush(stdout);
    }
    if (err < 0){ std::cout<<"Error reading input stream.";}

    err = Pa_CloseStream(stream);
    if (err != paNoError){ std::cout<<"Could not close input stream";}

}

Receiver::~Receiver() {
    cout <<"Deleting receiver object\n"; 
    Pa_Terminate();
}

// float* data will take in toneBuffer as an array of audio data. 
float goertzel_mag(int numSamples,int TARGET_FREQUENCY,int SAMPLING_RATE, float* data){
       
    float   q0 = 0 ,q1 = 0,q2 = 0; 
    float   scalingFactor = numSamples / 2.0;

    // w= (2*pi*k)/N
    // sine = sin(w) and cosine = cos(w)
    int k = (int) (0.5 + (((float)(numSamples * TARGET_FREQUENCY)) / SAMPLING_RATE));
    float omega = (2.0 * M_PI * k) / numSamples;
    float sine = sin(omega);
    float cosine = cos(omega);
    float coeff = 2.0 * cosine;
    
    for(int i=0; i<numSamples; i++)
    {
        q0 = coeff * q1 - q2 + data[i];
        q2 = q1;
        q1 = q0;
    }

    // calculate the real and imaginary results
    // scaling appropriately
    float real = (q1 - q2 * cosine) / scalingFactor;
    float imag = (q2 * sine) / scalingFactor;

    float magnitude = sqrt(real*real + imag*imag);
    return magnitude;
}

// There is a Hann function that mallocs an array dataOut. 
float* HannFunction(int numSamples, float* dataIn){
    
    float dataOut[BUFFER_SIZE];
    double multiplier; 
    for (int i =0; i< numSamples; ++i){
        multiplier = 0.5*(1-cos((2*M_PI*i)/(numSamples-1)));
        dataOut[i] = multiplier*dataIn[i];
    }
    return dataOut; 
}
