#ifndef AUDIO_RECEIVER_SRC_CONSTANTS_HPP_
#define AUDIO_RECEIVER_SRC_CONSTANTS_HPP_

// Port Audio Libraries
#include "portaudio.h"

// Audio Settings
#define SAMPLE_RATE  (44100) /* #define SAMPLE_RATE  (17932) // Test failure to open with this value. */
#define FRAMES_PER_BUFFER (512)
#define NUM_SECONDS     (5)
#define NUM_CHANNELS    (2)
#define DITHER_FLAG     (0) /* #define DITHER_FLAG     (paDitherOff) */

// Audio Format
#define PA_SAMPLE_TYPE  paFloat32
typedef float SAMPLE;
#define SAMPLE_SILENCE  (0.0f)
#define PRINTF_S_FORMAT "%.8f"

#endif /* AUDIO_RECEIVER_SRC_CONSTANTS_HPP_ */
