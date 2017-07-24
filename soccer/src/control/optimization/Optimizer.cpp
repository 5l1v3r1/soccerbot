#include "Optimizer.h"
#include <ctime>
#include <fstream>

//change to constructor later
Optimizer::Optimizer(NaoBehaviour*& behaviour_) {
    cout << "----------Initializing the optimizer----------" << endl;
    //behaviour = behaviour_;
	timePrevious = clock();
	COMPrevious = behaviour->outCenterOfMass();
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

//after to change fallen check in optimization
//after to change fallen check in optimization
//after to change fallen check in optimization
//after to change fallen check in optimization

//call the function in behaviout to detect fall
bool Optimizer::isFallen() {
    return behaviour->isFallen();
}

//just count time now
double Optimizer::countTime(){
	double interval = double( clock () - timePrevious ) /  CLOCKS_PER_SEC;
	timePrevious = clock();
	//cout << "Time interval is " << interval << endl;
	return interval;
}

//to output the center of mass for calculation
void Optimizer::outputAccerOfCOM(){
	double interval = countTime();
	VecPosition centerOfMassVec = behaviour->outCenterOfMass();
	VecPosition changeInCOM = centerOfMassVec - COMPrevious;
	COMPrevious = centerOfMassVec;
	//cout << changeInCOM << endl;
	//equation S = ut + 1/2* at^2 ???
	VecPosition acceOfCOM = (changeInCOM.getX(), changeInCOM.getY(), changeInCOM.getZ()) / pow(interval,2);
	cout << acceOfCOM << endl;
}
