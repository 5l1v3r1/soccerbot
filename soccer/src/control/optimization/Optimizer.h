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

protected:
    NaoBehaviour *behaviour;
    ofstream optimizing;

private:
};
