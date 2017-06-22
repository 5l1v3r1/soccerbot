//cpp file for checking the robot is fallen
#include "Optimizer.h"

//change to constructor later
Optimizer::Optimizer(NaoBehaviour*& behaviour_) {
    behaviour = behaviour_;
}

//destructor
Optimizer::~Optimizer() {
    delete behaviour;
}

bool Optimizer::isFallen() {
    return behaviour->isFallen();
}
