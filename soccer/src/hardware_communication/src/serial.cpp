#include "../include/hardware_communication/serial.h"
#include "../include/hardware_communication/packet.h"
#include <ros/console.h>
#include "string.h"

void send_goal(RobotGoal* robotGoal) {
	// Encapsulates the data into a packet
	Packet p;
	p.start_pattern = START_PATTERN;
	p.byte_count = sizeof(RobotGoal);
	memcpy(p.data, robotGoal, sizeof(RobotGoal));

	int wlen = write(fd, &p, sizeof(RobotGoal) + 16);
	if (wlen != sizeof(RobotGoal) + 16) {
		ROS_ERROR("Error from write: %d, %d\n", wlen, errno);
	}
	tcdrain(fd);
	tcflush(fd, TCIOFLUSH);
}

RobotState receive_state() {
	Packet p;
	RobotState state = {0};
	state.id = 0;

	memset(state.message,0,sizeof(state.message));

    char buffer[BUFFER_SIZE];
    uint BUFFER_INDEX = 0;
    u_int8_t start_char = (u_int8_t) 0xEE;
    u_int8_t* start_char_ptr = &start_char;
    u_int8_t data_size = 0;
    int dataread = 0;
    int n;

    ROS_ERROR("------------------------------");
    while(1) {
        n = read(fd, start_char_ptr, sizeof start_char);
        // Reading the start bit
        if (n > 0) {
            ROS_ERROR("%x", (unsigned char) start_char & 0xff);

            if(dataread == 0) {
				if ((unsigned char) start_char == START_PATTERN) {
					dataread = 1;
					ROS_ERROR("Start Byte");
				}
				continue;
            }

            // Reading the data size
            if(dataread == 1) {
				data_size = *start_char_ptr;
				if(data_size == 0) {
					dataread = 0;
					continue;
				}
				dataread = 2;
				ROS_ERROR("Data size %u bytes", data_size);
				continue;
            }

            // Read the actual message
            if(dataread == 2) {
            	data_size = data_size - 1;
            	buffer[BUFFER_INDEX++] = start_char;
            	if(data_size <= 0) {
            		memcpy(&state, buffer, sizeof(RobotState));
            		break;
            	}
            	continue;
            }
        }
    }
    ROS_ERROR("------------------------------");
    dataread = false;
	print_robot_state(state);
	tcflush(fd, TCIOFLUSH);
	return state;
}
