#include "receiver.h"
#include "portaudio.h"
#include <stdio.h>
using namespace std;

Receiver::Receiver() {
    int err = Pa_Initialize();
    if (err != paNoError) throw;

    inputParameters.device = Pa_GetDefaultInputDevice(); /* default input device */
    if (inputParameters.device == paNoDevice) {
        fprintf(stderr, "Error: No default input device.\n");
        Pa_Terminate();
        throw;
    }
    inputParameters.channelCount = 2; /* stereo input */
    inputParameters.sampleFormat = PA_SAMPLE_TYPE;
    inputParameters.suggestedLatency = Pa_GetDeviceInfo(inputParameters.device)->defaultLowInputLatency;
    inputParameters.hostApiSpecificStreamInfo = NULL;
}

Receiver::Receiver(const Receiver& orig) {
}

Receiver::~Receiver() {
}

static int recordCallback(  const void *inputBuffer, void *outputBuffer,
                            unsigned long framesPerBuffer,
                            const PaStreamCallbackTimeInfo* timeInfo,
                            PaStreamCallbackFlags statusFlags,
                            void *userData) {
    raw_audio* data = (raw_audio*) userData;
    const SAMPLE* rptr = (const SAMPLE*) inputBuffer;
    SAMPLE* wptr = &data->recordedSamples[data->frameIndex * NUM_CHANNELS];
    long framesToCalc;
    long i;
    int finished;
    unsigned long framesLeft = data->maxFrameIndex - data->frameIndex;

    (void) outputBuffer; /* Prevent unused variable warnings. */
    (void) timeInfo;
    (void) statusFlags;
    (void) userData;

    if (framesLeft < framesPerBuffer) {
        framesToCalc = framesLeft;
        finished = paComplete;
    } else {
        framesToCalc = framesPerBuffer;
        finished = paContinue;
    }

    if (inputBuffer == NULL) {
        for (i = 0; i < framesToCalc; i++) {
            *wptr++ = SAMPLE_SILENCE; /* left */
            if (NUM_CHANNELS == 2) *wptr++ = SAMPLE_SILENCE; /* right */
        }
    } else {
        for (i = 0; i < framesToCalc; i++) {
            *wptr++ = *rptr++; /* left */
            if (NUM_CHANNELS == 2) *wptr++ = *rptr++; /* right */
        }
    }
    data->frameIndex += framesToCalc;
    return finished;
}
/*
void Receiver::record_playback() {
    AudioPacket packet;
    audio_queue.push(packet);
    RawAudio* data = &(packet.data);
    
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
    if (err != paNoError) throw;

    err = Pa_StartStream(stream);
    if (err != paNoError) throw;
    printf("\n=== Now recording!! Please speak into the microphone. ===\n");
    fflush(stdout);

    while ((err = Pa_IsStreamActive(stream)) == 1) {
        Pa_Sleep(1000);
        printf("index = %d\n", data->frameIndex);
        fflush(stdout);
    }
    if (err < 0) throw;

    err = Pa_CloseStream(stream);
    if (err != paNoError) throw;

    // Measure maximum peak amplitude. 
    int max = 0;
    int average = 0.0;
    for (int i = 0; i < AudioPacket::numSamples; i++) {
        int val = data->recordedSamples[i];
        if (val < 0) val = -val; // ABS
        if (val > max) {
            max = val;
        }
        average += val;
    }

    average = average / (double) AudioPacket::numSamples;

    printf("sample max amplitude = "PRINTF_S_FORMAT"\n", max);
    printf("sample average = %lf\n", average);
}
*/