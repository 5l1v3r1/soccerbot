/*
 * RobotState.h
 *
 *  Created on: 2017-12-19
 *      Author: vuwij
 */

#ifndef HARDWARE_COMMUNICATION_INCLUDE_HARDWARE_COMMUNICATION_ROBOTSTATE_H_
#define HARDWARE_COMMUNICATION_INCLUDE_HARDWARE_COMMUNICATION_ROBOTSTATE_H_

#include <stdint.h>

typedef struct robotstate {
	uint32_t id;
	char message[12];
} RobotState;

void print_robot_state(RobotState& robotstate);

#endif /* HARDWARE_COMMUNICATION_INCLUDE_HARDWARE_COMMUNICATION_ROBOTSTATE_H_ */
