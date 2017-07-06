//h file to check the robot is fallen
#include "../behaviours/naobehaviour.h"

class Optimizer {
public:
    Optimizer(NaoBehaviour*& behaviour);
    ~Optimizer();
    bool isFallen();
    void countTime(clock_t beginTime);
    // updateBehaviour
    // runOptimization

protected:
    NaoBehaviour *behaviour;

private:
};
