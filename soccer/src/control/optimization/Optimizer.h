//h file to check the robot is fallen
#include "../behaviours/naobehaviour.h"
#include <fstream>
#include <ctime>

class Optimizer {
public:
    Optimizer(NaoBehaviour*& behaviour);
    ~Optimizer();
    bool isFallen();
    double countTime();
    // updateBehaviour
    // runOptimization
    void outputAccerOfCOM();
    void outputFrame(int currentKeyFrame, string skillName);

protected:
    NaoBehaviour *behaviour;
    //for output data
    ofstream optimizing;
    clock_t timePrevious;
    VecPosition COMPrevious;

private:
};
