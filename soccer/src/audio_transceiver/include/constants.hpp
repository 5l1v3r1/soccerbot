#ifndef AUDIO_RECEIVER_SRC_CONSTANTS_HPP_
#define AUDIO_RECEIVER_SRC_CONSTANTS_HPP_

// Port Audio Libraries
#include "portaudio.h"
#include <array>
#include <vector>

// Audio Settings
#define SAMPLE_RATE  (44100) /* #define SAMPLE_RATE  (17932) // Test failure to open with this value. */

// Earlier frames per buffer were 512 
// Logically should be 9600

// Assuming that each audio is a second long. 
#define FRAMES_PER_BUFFER (50)
#define NUM_CHANNELS    (2)
//#define DITHER_FLAG     (0) /* #define DITHER_FLAG     (paDitherOff) */
#define BUFFER_SIZE     (FRAMES_PER_BUFFER*NUM_CHANNELS) // defines the size. 4800 *2 (frames * number of channels) 
// Audio Format
#define PA_SAMPLE_TYPE  paFloat32
typedef float  SAMPLE;
#define SAMPLE_SILENCE  (0.0f)
#define PRINTF_S_FORMAT "%.8f"
#define NUMBER_OF_STEPS (5)

//Global constants

typedef struct raw_audio{
    int left_phase; 
    int right_phase; 
    // For some reason std:: array(C++11) doesn't work with vector whereas boost does so keeping that for now.
    std::vector< float > tone_buffer; 
    //std::vector< std::array<float, NUMBER_OF_STEPS> > tone_buffer; 
    //vector<float> tone_buffer[BUFFER_SIZE]; 
    //int frameIndex; /* Index into sample array. */
    //int maxFrameIndex;
    //SAMPLE *recordedSamples;
} RawAudio;

#endif /* AUDIO_RECEIVER_SRC_CONSTANTS_HPP_ */

