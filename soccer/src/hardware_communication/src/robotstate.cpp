#include <hardware_communication/robotstate.h>

void print_robot_state(RobotState& robotstate) {
	ROS_ERROR("----- ROBOT STATE ------");
//	char buf[sizeof(RobotState)+1];
//	memcpy(buf, (char *) &robotstate, sizeof(RobotState));
//	buf[sizeof(RobotState)] = '\n';
//	ROS_ERROR("%s", &buf);
	ROS_ERROR("%u - %s", robotstate.id, robotstate.message);
}

