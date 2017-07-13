//cpp file for checking the robot is fallen
#include "Optimizer.h"
#include <ctime>
#include <fstream>

//change to constructor later
Optimizer::Optimizer(NaoBehaviour*& behaviour_) {
    cout << "--------------Initializing the optimizer----------" << endl;
    behaviour = behaviour_;
    //to initialize the file 
    optimizing.open("./optimization/optimizing.txt");
    optimizing << "Starting output center of mass" << endl;
	if(!optimizing.is_open()){
		cout << "--------------Fail to initialize output file-------------" << endl;
	}
	else cout << "--------------Initializing successfully----------" << endl;
}

//destructor
Optimizer::~Optimizer() {
    delete behaviour;
    optimizing.close();
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
	VecPosition centerOfMassVec = behaviour->outCenterOfMass();
	optimizing << centerOfMassVec << endl;
}

/*
void Optimizer::outputFrame(int currentKeyFrame, string skillName){
	optimizing << "Finished keyframe " << currentKeyFrame << " of " << skillName << endl;
}
*/