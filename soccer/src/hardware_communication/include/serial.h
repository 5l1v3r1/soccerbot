#ifndef SERIAL_H
#define SERIAL_H

#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include "robotgoal.h"
#include "robotstate.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <termio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>

// Sends a goal message to the robot
void send_goal(RobotGoal* robotGoal);

// Receives a state message from the robot
RobotState receive_state();

#define START_PATTERN 0xAA
#define BUFFER_SIZE 1600
#define PACKET_SIZE 800

// FD
extern int fd;
extern RobotState robotState;
extern RobotGoal robotGoal, *robotGoalAck;

#endif
