#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "../behaviors/naobehavior.h"
#include <fstream>
#include <ctime>

class Optimizer {
public:
    Optimizer();
    ~Optimizer();
    bool isFallen();
    double countTime();
    // updateBehaviour
    // runOptimization
    void outputAccerOfCOM();

protected:
    //NaoBehaviour *behaviour;
    //for output data
    ofstream optimizing;
    clock_t timePrevious;
    VecPosition COMPrevious;

private:
};

#endif