#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H

#include <string>

class Behaviour {
public:

    Behaviour();
    virtual ~Behaviour();


    /** called once when the initially connected to the server */
    virtual std::string Init() = 0;

    /** called for every message received from the server; should
        return an action string
     */
    virtual std::string Think(const std::string& message) = 0;

    /** Get message for sending to the server through the monitor port */
    virtual std::string getMonMessage() = 0;
};


#endif