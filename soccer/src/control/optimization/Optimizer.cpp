//cpp file for checking the robot is fallen
#include "Optimizer.h"
#include <ctime>
#include <fstream>

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

//to output the center of mass for calculation
void Optimizer::outputCenterOfMass(){
	std::ofstream optimizing;
	optimizing.open("./optimizing.txt");
	if(!optimizing.is_open()){
		cout << "fail to the output file" << endl;
	}
	VecPosition centerOfMassVec = behaviour->outCenterOfMass();
	optimizing << centerOfMassVec << endl;

}