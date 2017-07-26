#include "naobehaviour.h"
#include "../headers/headers.h"
#include "../skills/skillparser.h"
//#include "../optimization/Optimizer.h"

NaoBehaviour::NaoBehaviour(const std::string teamName, int uNum, const map <std::string, std::string>& namedParams_, const std::string& rsg_) :
namedParams(namedParams_), rsg(rsg_) {

    //note here the file is connet with main.cpp so the output/input file is created under control not in control/skill 
    //and also add the skill name below"skillType arr"                 
    readSkillsFromFile("./skills/stand.skl");
    readSkillsFromFile("./skills/test.skl");
    readSkillsFromFile("./skills/walk.skl");


    worldModel = new WorldModel();
    bodyModel = new BodyModel(worldModel);
    //parser has same body and world model with naobehavior
    parser = new Parser(worldModel, bodyModel);


    //chage to optimizer later
    timePrevious = returnTimeInSecond();
    COMPrevious = bodyModel->getCenterOfMass();
    acceOfCOM = (0, 0, 0);
    
    
    //add the new skill here !!
    //static const SkillType arr[] = {SKILL_STAND, SKILL_WALK, SKILL_WALK, SKILL_WALK, SKILL_WALK, SKILL_WALK, SKILL_WALK};
    static const SkillType arr[] = {SKILL_TEST};
    skillSequence = vector<SkillType>(arr, arr + sizeof (arr) / sizeof (arr[0]));
    currentSkillIndex = 0;
}

NaoBehaviour::~NaoBehaviour() {
    delete parser;
    delete bodyModel;
    delete worldModel;
}

std::string NaoBehaviour::Init() {
    return "(scene " + rsg + ")";
}

std::string NaoBehaviour::Think(const std::string& message) {
    bool parseSuccess = parser->parse(message);
    bodyModel->refresh();
    //below is the skill type
    boost::shared_ptr<Skill> skillToExecute = skills[skillSequence[currentSkillIndex]];
    // Loop through the skill sequence
    bool checkFinishSkill = skillToExecute->execute(bodyModel, worldModel);

    //Ok here if you cannot insert inside the another class you can let it reutrn a value to upper loop
    //if(skillToExecute->checkFinishOfKeyFrame()){
    //    outputAccerOfCOM();
    //}
    //here now the keyframe are too few so the time interval is relatively large refresh time set to 0.5s
    if((returnTimeInSecond()/* - timePrevious*/) < 11){
        calcChangeOfCOM();
    }
    //check fallen state
    isFallen();




    if (checkFinishSkill == true) {
        std::cout << "Finished executing " << skillToExecute->getName() << std::endl;
        skillToExecute->reset();
        currentSkillIndex += 1;
        currentSkillIndex %= skillSequence.size();
    }

    worldModel->setLastSkill(skillSequence[currentSkillIndex]);
    std::string action = composeAction();

    //hide the center of mass
    //std::cout << "The center of mass is "<< bodyModel->getCenterOfMass() << std::endl;
    //cout << "The location of agent is " << worldModel->getMyPosition() << endl;
    return action;
}

std::string NaoBehaviour::composeAction() {
    stringstream ss("");

    ss << "(he1 " << bodyModel->computeTorque(EFF_H1) << ")";
    ss << "(he2 " << bodyModel->computeTorque(EFF_H2) << ")";

    ss << "(lae1 " << bodyModel->computeTorque(EFF_LA1) << ")";
    ss << "(lae2 " << bodyModel->computeTorque(EFF_LA2) << ")";
    ss << "(lae3 " << bodyModel->computeTorque(EFF_LA3) << ")";
    ss << "(lae4 " << bodyModel->computeTorque(EFF_LA4) << ")";

    ss << "(rae1 " << bodyModel->computeTorque(EFF_RA1) << ")";
    ss << "(rae2 " << bodyModel->computeTorque(EFF_RA2) << ")";
    ss << "(rae3 " << bodyModel->computeTorque(EFF_RA3) << ")";
    ss << "(rae4 " << bodyModel->computeTorque(EFF_RA4) << ")";

    ss << "(lle1 " << bodyModel->computeTorque(EFF_LL1) << ")";
    ss << "(lle2 " << bodyModel->computeTorque(EFF_LL2) << ")";
    ss << "(lle3 " << bodyModel->computeTorque(EFF_LL3) << ")";
    ss << "(lle4 " << bodyModel->computeTorque(EFF_LL4) << ")";
    ss << "(lle5 " << bodyModel->computeTorque(EFF_LL5) << ")";
    ss << "(lle6 " << bodyModel->computeTorque(EFF_LL6) << ")";
    if (bodyModel->hasToe()) {
        ss << "(lle7 " << bodyModel->computeTorque(EFF_LL7) << ")";
    }

    ss << "(rle1 " << bodyModel->computeTorque(EFF_RL1) << ")";
    ss << "(rle2 " << bodyModel->computeTorque(EFF_RL2) << ")";
    ss << "(rle3 " << bodyModel->computeTorque(EFF_RL3) << ")";
    ss << "(rle4 " << bodyModel->computeTorque(EFF_RL4) << ")";
    ss << "(rle5 " << bodyModel->computeTorque(EFF_RL5) << ")";
    ss << "(rle6 " << bodyModel->computeTorque(EFF_RL6) << ")";
    if (bodyModel->hasToe()) {
        ss << "(rle7 " << bodyModel->computeTorque(EFF_RL7) << ")";
    }

    return ss.str();
}

std::string NaoBehaviour::getMonMessage() {
    return "";
}

void NaoBehaviour::readSkillsFromFile(const std::string& filename) {
    SkillParser skillParser(skills, bodyModel);
    string skillDescription = skillParser.preprocess(filename, namedParams);
    parse_info<iterator_t> info = parse(skillDescription.c_str(),
            skillParser,
            (space_p | comment_p("#"))
            );
    if (!info.hit) {
        std::cout << "Parsing skill " << filename << " failed" << std::endl;
    }
}

//to check if the agent is fallen down
bool NaoBehaviour::isFallen() {
    //fallen backward
    if (returnTimeInSecond() < 3) return false;
    //for begining state
    else if(calcChangeOfCOM().getX() < -0.001 && returnTimeInSecond() > 3) {
        cout << "I am fallen backward!" << endl;
        return true;
    }
    else return false;
}

VecPosition NaoBehaviour::outCenterOfMass(){
    return bodyModel->getCenterOfMass();
}


//return time in seconds but this is little bit faster
double NaoBehaviour::returnTimeInSecond(){
    return (double(clock ()) /  CLOCKS_PER_SEC * 100);
}

//to output the center of mass for calculation
VecPosition NaoBehaviour::calcChangeOfCOM(){
    timePrevious = returnTimeInSecond();
    VecPosition centerOfMassVec = outCenterOfMass();
    VecPosition changeOfCOM = centerOfMassVec - COMPrevious;
    COMPrevious = centerOfMassVec;
    //equation S = ut + 1/2* at^2 ???
    //accerOf Z direction is very small
    //acceOfCOM = (changeOfCOM.getX(), changeOfCOM.getY(), changeOfCOM.getZ()) * 2 / pow(interval,2);
    cout << changeOfCOM << endl;
    return changeOfCOM;
}

