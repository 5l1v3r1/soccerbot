#include "serial.h"
#include "string.h"

using namespace std;

void send_goal(RobotGoal* robotGoal) {
	// Encapsulates the data into a packet
	int wlen = write(fd, robotGoal, sizeof(RobotGoal));
	if (wlen != sizeof(RobotGoal)) {
		printf("Error from write: %d, %d\n", wlen, errno);
	}
	tcflush(fd, TCOFLUSH);
}

RobotState receive_state() {
	RobotState state = {0};
	state.id = 0;
	memset(state.message,0,sizeof(state.message));

	while(1) {
        int n = read(fd, &state, sizeof (RobotState)-1);

        if(n > 0) {
        	tcflush(fd, TCIFLUSH);
        	break;
        }
        if(n == 0) {
        	usleep(200000);
        	cout << "Sleep" << endl;
        }
        if(n < 0) {
        	cout << "Error" << endl;
        }
    }
	return state;
}
