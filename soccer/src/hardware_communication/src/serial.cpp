#include "../include/hardware_communication/serial.h"

void send_goal(RobotGoal* robotGoal) {
	int wlen = write(fd, (char *) robotGoal, sizeof(RobotGoal));
	if (wlen != sizeof(RobotGoal)) {
		ROS_ERROR("Error from write: %d, %d\n", wlen, errno);
	}
	tcdrain(fd);
	tcflush(fd, TCIOFLUSH);
}

RobotState receive_state() {
	RobotState state = {0};
	state.id = 0;
	memset(state.message,0,sizeof(state.message));

	int rdlen = 0;
	while (rdlen != sizeof(RobotState)) {
		rdlen = read(fd, &state, sizeof(RobotState));
		if (rdlen > 0) {
			break;
		} else if (rdlen < 0) {
			printf("Error from read: %d: %s\n", rdlen, strerror(errno));
		}
	}
	print_robot_state(state);
	tcflush(fd, TCIOFLUSH);
	return state;
}
