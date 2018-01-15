#include "robotstate.h"
#include <stdio.h>

void print_robot_state(RobotState& robotstate) {
	printf("----- ROBOT STATE ------");
//	char buf[sizeof(RobotState)+1];
//	memcpy(buf, (char *) &robotstate, sizeof(RobotState));
//	buf[sizeof(RobotState)] = '\n';
//	ROS_ERROR("%s", &buf);
	printf("%u - %s", robotstate.id, robotstate.message);
}

