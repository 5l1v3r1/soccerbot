#include "naobehaviour.h"

#include "../headers/headers.h"
#include "../skills/skillparser.h"

NaoBehaviour::NaoBehaviour(const std::string teamName, int uNum, const map <std::string, std::string>& namedParams_, const std::string& rsg_) :
namedParams(namedParams_), rsg(rsg_) {	

    readSkillsFromFile("./skills/stand.skl");
	
	readSkillsFromFile("./skills/wave.skl");

    worldModel = new WorldModel();
    bodyModel = new BodyModel(worldModel);
    parser = new Parser(worldModel, bodyModel);

    static const SkillType arr[] = {SKILL_STAND};
    skillSequence = vector<SkillType>(arr, arr + sizeof(arr) / sizeof(arr[0]));
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
    boost::shared_ptr<Skill> skillToExecute = skills[skillSequence[currentSkillIndex]];

    // Loop through the skill sequence
    if(skillToExecute->execute(bodyModel, worldModel)) {
        std::cout << "Finished the skill!" << std::endl;
        skillToExecute->reset();
        currentSkillIndex += 1;
        currentSkillIndex %= skillSequence.size();
    }
    worldModel->setLastSkill(skillSequence[currentSkillIndex]);
    std::string action = composeAction();
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

void NaoBehaviour::readSkillsFromFile( const std::string& filename) {
    SkillParser skillParser(skills, bodyModel);
    string skillDescription = 
        skillParser.preprocess(filename, namedParams);
    parse_info<iterator_t> info = parse(skillDescription.c_str(),
            skillParser,
            (space_p | comment_p("#"))
            );
    if (!info.hit) {
        std::cout << "Parsing skill " << filename << " failed" << std::endl;
    }
}

