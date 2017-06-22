//h file to check the robot is fallen
#include "../behaviours/naobehaviour.h"

class Optimizer {
public:
    Optimizer(NaoBehaviour*& behaviour);
    ~Optimizer();
    bool isFallen();
    // updateBehaviour
    // runOptimization

protected:
    NaoBehaviour *behaviour;

private:
};
