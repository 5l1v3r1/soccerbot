#ifndef AUDIO_RECEIVER_SRC_CONSTANTS_HPP_
#define AUDIO_RECEIVER_SRC_CONSTANTS_HPP_

// Port Audio Libraries
#include "portaudio.h"

// Audio Settings
#define SAMPLE_RATE  (48000) /* #define SAMPLE_RATE  (17932) // Test failure to open with this value. */

#define FRAMES_PER_BUFFER (2400)
// Earlier frames per buffer were 512 
// Logically should be 2400
#define NUM_SECONDS     (5)
#define NUM_CHANNELS    (2)
#define DITHER_FLAG     (0) /* #define DITHER_FLAG     (paDitherOff) */
#define BUFFER_SIZE     (9600) // defines the size. 4800 *2 (samples * number of channels) 
// Audio Format
#define PA_SAMPLE_TYPE  paFloat32
typedef float SAMPLE;
#define SAMPLE_SILENCE  (0.0f)
#define PRINTF_S_FORMAT "%.8f"


//Global constants

typedef struct raw_audio{
    int left_phase; 
    int right_phase; 
    float tone_buffer[BUFFER_SIZE]; 
    int frameIndex; /* Index into sample array. */
    int maxFrameIndex;
    SAMPLE *recordedSamples;
} RawAudio;

#endif /* AUDIO_RECEIVER_SRC_CONSTANTS_HPP_ */
