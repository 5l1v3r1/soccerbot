#include <cmath>

#include "parser.h"
#include "../worldmodel/WorldObject.h"

using boost::shared_ptr;

Parser::Parser(WorldModel *worldModel, BodyModel *bodyModel) {
    this->worldModel = worldModel;
    this->bodyModel = bodyModel;
}

Parser::~Parser() {}

vector<string> Parser::tokenise(const string &s) {
    int length = s.length();
    string currentString = "";
    bool currentValid = false;

    vector<string> v;

    for(int i = 0; i < length; ++i) {
        char c = s.at(i);
        if(c != '(' && c != ')' && c != ' ') {
            currentString.append(1, c);
        }
        else {

            if(currentString.length() > 0) {
                v.push_back(currentString);
                currentString = "";
            }
        }
    }

    return v;
}

bool Parser::parseTime(const string &str) {

    bool valid = false;
    double time = 0;
    vector<string> tokens = tokenise(str);
    for(size_t i = 0; i < tokens.size() - 1; ++i) {
        if(!(tokens[i].compare("now"))) {
            time = atof(tokens[i + 1].c_str());
            valid = true;
        }
    }

    if(valid) {
        worldModel->setTime(time);
        worldModel->incrementCycle();
    }

    return valid;
}

bool Parser::parseHingeJoint(const string &str) {

    bool valid;

    string name;
    double angle = 0.0;

    bool validName = false;
    bool validAngle = false;

    int hingeJointIndexBM = -1;

    vector<string> tokens = tokenise(str);
    for(size_t i = 0; i < tokens.size(); ++i) {

        if(!tokens[i].compare("n")) {
            if(i + 1 < tokens.size()) {

                name = tokens[i + 1];

                if(!(name.compare("hj1"))) {
                    hingeJointIndexBM = HJ_H1;
                    validName = true;
                }
                else if(!(name.compare("hj2"))) {
                    hingeJointIndexBM = HJ_H2;
                    validName = true;
                }
                else if(!(name.compare("laj1"))) {
                    hingeJointIndexBM = HJ_LA1;
                    validName = true;
                }
                else if(!(name.compare("laj2"))) {
                    hingeJointIndexBM = HJ_LA2;
                    validName = true;
                }
                else if(!(name.compare("laj3"))) {
                    hingeJointIndexBM = HJ_LA3;
                    validName = true;
                }
                else if(!(name.compare("laj4"))) {
                    hingeJointIndexBM = HJ_LA4;
                    validName = true;
                }
                else if(!(name.compare("raj1"))) {
                    hingeJointIndexBM = HJ_RA1;
                    validName = true;
                }
                else if(!(name.compare("raj2"))) {
                    hingeJointIndexBM = HJ_RA2;
                    validName = true;
                }
                else if(!(name.compare("raj3"))) {
                    hingeJointIndexBM = HJ_RA3;
                    validName = true;
                }
                else if(!(name.compare("raj4"))) {
                    hingeJointIndexBM = HJ_RA4;
                    validName = true;
                }
                else if(!(name.compare("llj1"))) {
                    hingeJointIndexBM = HJ_LL1;
                    validName = true;
                }
                else if(!(name.compare("llj2"))) {
                    hingeJointIndexBM = HJ_LL2;
                    validName = true;
                }
                else if(!(name.compare("llj3"))) {
                    hingeJointIndexBM = HJ_LL3;
                    validName = true;
                }
                else if(!(name.compare("llj4"))) {
                    hingeJointIndexBM = HJ_LL4;
                    validName = true;
                }
                else if(!(name.compare("llj5"))) {
                    hingeJointIndexBM = HJ_LL5;
                    validName = true;
                }
                else if(!(name.compare("llj6"))) {
                    hingeJointIndexBM = HJ_LL6;
                    validName = true;
                }
                else if(!(name.compare("llj7"))) {
                    hingeJointIndexBM = HJ_LL7;
                    validName = true;
                }
                else if(!(name.compare("rlj1"))) {
                    hingeJointIndexBM = HJ_RL1;
                    validName = true;
                }
                else if(!(name.compare("rlj2"))) {
                    hingeJointIndexBM = HJ_RL2;
                    validName = true;
                }
                else if(!(name.compare("rlj3"))) {
                    hingeJointIndexBM = HJ_RL3;
                    validName = true;
                }
                else if(!(name.compare("rlj4"))) {
                    hingeJointIndexBM = HJ_RL4;
                    validName = true;
                }
                else if(!(name.compare("rlj5"))) {
                    hingeJointIndexBM = HJ_RL5;
                    validName = true;
                }
                else if(!(name.compare("rlj6"))) {
                    hingeJointIndexBM = HJ_RL6;
                    validName = true;
                }
                else if(!(name.compare("rlj7"))) {
                    hingeJointIndexBM = HJ_RL7;
                    validName = true;
                }
            }
        }

        if(!tokens[i].compare("ax")) {
            if(i + 1 < tokens.size()) {

                angle = atof(tokens[i + 1].c_str());
                validAngle = true;
            }
        }

    }

    valid = validName && validAngle;

    if(valid) {
        bodyModel->setJointAngle(hingeJointIndexBM, angle);
    }

    return valid;
}

vector<string> Parser::segment(const string &str, const bool &omitEnds) {
    vector<string> v;

    int ptr = 0;
    int length = str.length();

    if(omitEnds) {
        ptr = 1;
        length = str.length() - 1;
    }

    int bracCount = 0;

    string currentString = "";

    do {

        while(ptr < length && str.at(ptr) != '(') {
            ptr++;
        }

        if(ptr < length) {

            currentString = "";

            do {
                char c = str.at(ptr);

                if(c == '(') {
                    bracCount++;
                }
                else if(c == ')') {
                    bracCount--;
                }

                currentString.append(1, c);
                ptr++;

            } while(bracCount != 0 && ptr < length);

            if(bracCount == 0) {
                v.push_back(currentString);
                currentString = "";
            }
        }
    } while(ptr < length);

    return v;
}

bool Parser::parse(const string &input) {
    bool valid = true;
    vector<string> inputSegments = segment(input, false);

    for(size_t i = 0; i < inputSegments.size(); ++i) {
        //Time
        if(inputSegments[i].at(1)== 't') {
            valid = parseTime(inputSegments[i]) && valid;
        }
        //Hinge Joint
        else if(inputSegments[i].at(1) == 'H') {
            valid = parseHingeJoint(inputSegments[i]) && valid;
        }
    }

    return valid;
}
