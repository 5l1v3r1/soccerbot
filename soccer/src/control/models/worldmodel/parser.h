#ifndef PARSER_H
#define PARSER_H

#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <boost/shared_ptr.hpp>

class WorldModel;
class BodyModel;
class VecPosition;

#include "../../headers/headers.h"
#include "worldmodel.h"
#include "../bodymodel/bodymodel.h"
#include "WorldObject.h"

using namespace std;

class Parser {
private:

    WorldModel *worldModel;
    BodyModel *bodyModel;

    int uNum;
    int side;

    string teamName;

    bool fProcessedVision;

protected:
    vector<string> tokenise(const string &s);
    vector<string> segment(const string &str, const bool &omitEnds);
    bool parseTime(const string &str);
    bool parseHingeJoint(const string &str);
    bool parseMyPos(const string &str);
    bool parseBallPos(const string &str);
public:
    Parser(WorldModel *worldModel, BodyModel *bodyModel);
    ~Parser();
    bool parse(const string &input);
};

#endif // PARSER_H

