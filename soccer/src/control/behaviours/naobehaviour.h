#ifndef NAOBEHAVIOUR_H
#define NAOBEHAVIOUR_H

#include "../headers/headers.h"
#include "behaviour.h"
#include "../models/worldmodel/worldmodel.h"
#include "../models/worldmodel/parser.h"
#include "../models/bodymodel/bodymodel.h"
#include "../skills/skill.h"

class NaoBehaviour : public Behaviour {
protected:
    map< SkillType, boost::shared_ptr<Skill> > skills;
    const map<string, string>& namedParams;
    string rsg;
    BodyModel *bodyModel;
    WorldModel *worldModel;
    Parser *parser;

    vector<SkillType> skillSequence;
    int currentSkillIndex;

    void readSkillsFromFile(const std::string& filename);
    std::string composeAction();

public:
    NaoBehaviour(const std::string teamName, int uNum, const map <std::string, std::string>& namedParams_, const std::string& rsg_);
    virtual ~NaoBehaviour();

    virtual std::string Init();
    virtual std::string Think(const std::string& message);

    bool isFallen();

    void setMonMessage(const std::string& msg);
    std::string getMonMessage();
};

#endif
