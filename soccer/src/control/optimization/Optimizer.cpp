#include "Optimizer.h"

Optimizer::Optimizer(NaoBehaviour*& behaviour_) {
    behaviour = behaviour_;
}

Optimizer::~Optimizer() {
    delete behaviour;
}

bool Optimizer::isFallen() {
    return behaviour->isFallen();
}
