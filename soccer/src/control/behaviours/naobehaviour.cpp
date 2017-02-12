#include "naobehaviour.h"

#include "../headers/headers.h"
#include "../skills/skillparser.h"

NaoBehaviour::NaoBehaviour(const std::string teamName, int uNum, const map <std::string, std::string>& namedParams_, const std::string& rsg_) :
namedParams(namedParams_), rsg(rsg_) {	

    readSkillsFromFile("./skills/stand.skl");

    worldModel = new WorldModel();
    bodyModel = new BodyModel(worldModel);
    skill = SKILL_STAND;
}

NaoBehaviour::~NaoBehaviour() {
}

std::string NaoBehaviour::Init() {
    return "(scene " + rsg + ")";
}

std::string NaoBehaviour::Think(const std::string& message) {
	return "";
}

std::string NaoBehaviour::getMonMessage() {
	return "";
}

void NaoBehaviour::readSkillsFromFile( const std::string& filename) {
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

