#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include <audio_packet.hpp>

class Transmitter {
public:
    bool send_message(AudioPacket& packet);
    
    Transmitter();
    Transmitter(const Transmitter& orig);
    virtual ~Transmitter();
private:

};

#endif /* TRANSMITTER_H */

