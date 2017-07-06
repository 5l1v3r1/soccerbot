//cpp file for checking the robot is fallen
#include "Optimizer.h"
#include <ctime>

//change to constructor later
Optimizer::Optimizer(NaoBehaviour*& behaviour_) {
    behaviour = behaviour_;
}

//destructor
Optimizer::~Optimizer() {
    delete behaviour;
}

//call the function in behaviout to detect fall
bool Optimizer::isFallen() {
    return behaviour->isFallen();
}

//just count time now
void Optimizer::countTime(clock_t beginTime){
	std::cout << float( clock () - beginTime ) /  CLOCKS_PER_SEC << endl;
}
