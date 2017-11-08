#include "../../audio/include/audio_packet.hpp"

#include <vector>


static const double PI = 3.14157; 
static const double FADE_LENGTH = 1;
//int AudioPacket::numSamples = 0;
// Helper function. A lot of improvements to be done in the code written.
std::array<float,NUMBER_OF_STEPS> sweep(double f_start, double f_end, double interval); 


AudioPacket::AudioPacket() {
    // for each pair of low and high frequencies
    // f1 is basically a value out of the low freq 
    // f2 is basically a value out of the high freq 
    lower_freq  = 1; 
    higher_freq = 5;
    //totalFrames = 100;  
    data.left_phase  = 0;
    data.right_phase = 0; 
    
    
    std::array<float,NUMBER_OF_STEPS>  tempArray; 
    //BUFFER SIZE IS 88200
    for (int k = 0; k < BUFFER_SIZE; k++) {
        float f1 = lower_freq;
        float f2 = higher_freq;
        // calculating the total samples. 
        // calculate the angle for a 1hz signal
        float unit_hz = 2 * PI * k / SAMPLE_RATE;

        // compute k_th sample, the sum of both signals at time x
        // amp_low and amp_high are adjustable magnitude coefficients for each frequency 
        //tone_buffer[k][i][j] =((short)amp_low [i] * sin(f1 * unit_hz)) + ((short)amp_high[j] * sin(f2 * unit_hz));
        // Modified sweep function that has been delivered.
        int index = k*NUM_CHANNELS;
        for (int i = 0; i < NUM_CHANNELS; i++){
            data.tone_buffer.push_back((float) sin( ((double)index/((double)BUFFER_SIZE)) * M_PI * 2.0 ));
        }
        
        //tempArray = sweep(f1,f2,unit_hz);
        for(int i = 0; i < data.tone_buffer.size(); i++) {
            cout << data.tone_buffer[i] << " ";
        }
        cout << endl;
        
        //for (int i = 0; i < NUM_CHANNELS; i++){
        //    data.tone_buffer.push_back(tempArray);
        //}

        // fade out samples near the end of the buffer
        float dist_to_end = (BUFFER_SIZE - k)/BUFFER_SIZE;
        float fade = 1.0;
        if (dist_to_end < FADE_LENGTH) {
            fade = dist_to_end / FADE_LENGTH;
        }
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

//void AudioPacket::convert_message_to_raw_audio(){ 
    
//}

std::array<float,NUMBER_OF_STEPS> sweep(double f_start, double f_end, double interval) {
    std::array<float,NUMBER_OF_STEPS> tempAudio; 
    for (int i = 1; i <= NUMBER_OF_STEPS; i++) {
        double delta = i/(double)NUMBER_OF_STEPS;
        
        double t = interval * delta; 
        double phase = 2 * PI * t * (f_start + (f_end - f_start) * delta / 2);
        while (phase > 2 * PI) phase -= 2 * PI; // optional
        tempAudio[i] = (10 * sin(phase));
    }
    return tempAudio;
}

RawAudio& AudioPacket::get_data(){
    return data; 
}