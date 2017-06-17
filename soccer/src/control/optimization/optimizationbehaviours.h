#ifndef _OPTIMIZATION_BEHAVIOURS_H
#define _OPTIMIZATION_BEHAVIOURS_H

#include "../behaviors/naobehaviour.h"

bool isBallMoving(const WorldModel *worldModel);

class OptimizationBehaviourFixedKick : public NaoBehaviour {
    const string outputFile;

    double timeStart;
    bool hasKicked;
    bool beamChecked;
    bool backwards;
    bool ranIntoBall;
    bool fallen;

    int kick;

    double INIT_WAIT_TIME;

    VecPosition ballInitPos;
    void initKick();
    void writeFitnessToOutputFile(double fitness);

public:

    OptimizationBehaviourFixedKick(const std::string teamName, int uNum, const map<
            string, string>& namedParams_, const string& rsg_, const string& outputFile_);

    virtual void beam(double& beamX, double& beamY, double& beamAngle);
    virtual SkillType selectSkill();
    virtual void updateFitness();

};

class OptimizationBehaviourWalkForward : public NaoBehaviour {
    const string outputFile;

    int run;
    double startTime;
    bool beamChecked;
    double INIT_WAIT;
    double totalWalkDist;

    void init();
    bool checkBeam();

public:

    OptimizationBehaviourWalkForward(const std::string teamName, int uNum, const map<string, string>& namedParams_, const string& rsg_, const string& outputFile_);

    virtual void beam(double& beamX, double& beamY, double& beamAngle);
    virtual SkillType selectSkill();
    virtual void updateFitness();

};

#endif
