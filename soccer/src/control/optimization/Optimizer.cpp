//cpp file for checking the robot is fallen
#include "Optimizer.h"
#include <ctime>
#include <fstream>

//change to constructor later
Optimizer::Optimizer(NaoBehaviour*& behaviour_) {
    cout << "----------Initializing the optimizer----------" << endl;
    behaviour = behaviour_;
	time = clock();
    //to initialize the output file 
    optimizing.open("./optimization/optimizing.txt");
    optimizing << "Starting output center of mass" << endl;
	if(!optimizing.is_open()){
		cout << "----------Fail to initialize output file----------" << endl;
	}
	else cout << "----------Initializing successfully----------" << endl;
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
void Optimizer::countTime(){
	cout << "Time interval is " << float( clock () - time ) /  CLOCKS_PER_SEC << endl;
	//refresh the time 
	time = clock ();
}

//to output the center of mass for calculation
void Optimizer::outputAccerOfCOM(){
	VecPosition centerOfMassVec = behaviour->outCenterOfMass();
	optimizing << centerOfMassVec << endl;
}

/*
void Optimizer::outputFrame(int currentKeyFrame, string skillName){
	optimizing << "Finished keyframe " << currentKeyFrame << " of " << skillName << endl;
}
*/