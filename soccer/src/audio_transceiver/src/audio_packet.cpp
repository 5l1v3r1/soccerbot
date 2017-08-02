#include "audio_packet.hpp"
#include <math.h>
static const double PI = 3.14157; 
static const double FADE_LENGTH = 1;
//int AudioPacket::numSamples = 0;
// Helper function. A lot of improvements to be done in the code written.
float sweep(double f_start, double f_end, double interval, int n_steps); 


// Very poor code. Because copy paste. 
AudioPacket::AudioPacket() {
    // for each pair of low and high frequencies
    // f1 is basically a value out of the low freq 
    // f2 is basically a value out of the high freq 
    int total_samples = numSamples * numChannels; 
    for( int k = 0; k < numSamples; k++){ 
        for(int  i = 0; i < num_low_freqs; i++) {
            for(int j = 0; j < num_high_freqs; j++) {
                float f1 = lower_freq[i];
                float f2 = higher_freq[j];
                // calculating the total samples. 
                // calculate the angle for a 1hz signal
                float unit_hz = 2 * PI * k / SAMPLE_RATE;

                // compute k_th sample, the sum of both signals at time x
                // amp_low and amp_high are adjustable magnitude coefficients for each frequency 
                //tone_buffer[k][i][j] =((short)amp_low [i] * sin(f1 * unit_hz)) + ((short)amp_high[j] * sin(f2 * unit_hz));
                // Modified sweep function that has been delivered.
                tone_buffer[(k*numChannels)][i][j] = sweep(f1,f2, 1.2, 5);
                // Random interval passed right now.
                // Random steps.
                //sweep(f1,f2,1.2, 5)
                // fade out samples near the end of the buffer
                float dist_to_end = total_samples - k;
                float fade = 1.0;
                if(dist_to_end < FADE_LENGTH){
                    fade = dist_to_end/FADE_LENGTH;
                }
                tone_buffer[(k*numChannels)][i][j] *= fade;
                // copy k_th sample to the remaining channels
                for(int l = (k*numChannels) + 1; l < (k*numChannels) + numChannels; l++) {
                    tone_buffer[l][i][j] = tone_buffer[(k*numChannels)][i][j];
                }
            }
        }
    }
}

float sweep(double f_start, double f_end, double interval, int n_steps) {
    for (int i = 0; i < n_steps; ++i) {
        double delta = i / (float)n_steps;
        double t = interval * delta;
        double phase = 2 * PI * t * (f_start + (f_end - f_start) * delta / 2);
        while (phase > 2 * PI) phase -= 2 * PI; // optional
        return 3 * sin(phase);
    }
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

void AudioPacket::convert_message_to_raw_audio(){ 
    
}

