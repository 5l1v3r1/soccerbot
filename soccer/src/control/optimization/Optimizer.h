//h file to check the robot is fallen
#include "../behaviours/naobehaviour.h"
#include <fstream>

class Optimizer {
public:
    Optimizer(NaoBehaviour*& behaviour);
    ~Optimizer();
    bool isFallen();
    void countTime(clock_t beginTime);
    // updateBehaviour
    // runOptimization
    void outputCenterOfMass();
    void outputFrame(int currentKeyFrame, string skillName);

protected:
    NaoBehaviour *behaviour;
    //for output data
    ofstream optimizing;

private:
};
