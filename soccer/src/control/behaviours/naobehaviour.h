#ifndef NAOBEHAVIOUR_H
#define NAOBEHAVIOUR_H

#include "../headers/headers.h"
#include "behaviour.h"

class NaoBehaviour : public Behaviour {

public:

    NaoBehaviour(const std::string teamName, int uNum, const map <std::string, std::string>& namedParams_, const std::string& rsg_);
    virtual ~NaoBehaviour();

    virtual std::string Init();
    virtual std::string Think(const std::string& message);

    void setMonMessage(const std::string& msg);
    std::string getMonMessage();

    
};

#endif