#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include <hardware_communication/robotgoal.h>
#include <hardware_communication/robotstate.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <termio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>
#include "../include/hardware_communication/serial.h"

using namespace std;
using namespace ros;

int fd;
RobotState robotState;
RobotGoal robotGoal, *robotGoalPtr;

int open_port(void) {
	int fd; // file description for the serial port

	fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);

	if(fd == -1)
		fd = open("/dev/ttyACM1", O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd == -1)
		fd = open("/dev/ttyACM2", O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd == -1)
		fd = open("/dev/ttyACM3", O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd == -1)
		fd = open("/dev/ttyACM4", O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd == -1)
		fd = open("/dev/ttyACM5", O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd == -1)
		fd = open("/dev/ttyACM6", O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd == -1)
		fd = open("/dev/ttyACM7", O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd == -1)
		fd = open("/dev/ttyACM8", O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd == -1)
		fd = open("/dev/ttyACM9", O_RDWR | O_NOCTTY | O_NDELAY);

	if (fd == -1) {
		ROS_ERROR("open_port: Unable to open /dev/ttyACM0. \n");
		exit(1);
	} else {
		fcntl(fd, F_SETFL, 0);
		ROS_ERROR("port is open.\n");
	}

	return (fd);
} //open_port

int configure_port(int fd)      // configure the port
		{
	struct termios port_settings;     // structure to store the port settings in

	cfsetispeed(&port_settings, B115200);    // set baud rates
	cfsetospeed(&port_settings, B115200);

	port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, data bits
	port_settings.c_cflag &= ~CSTOPB;
	port_settings.c_cflag &= ~CSIZE;
	port_settings.c_cflag |= CS8;

	tcsetattr(fd, TCSANOW, &port_settings);    // apply the settings to the port
	return (fd);

} //configure_port

int query_modem(int fd)   // query modem with an AT command
		{
	char n;
	fd_set rdfs;
	struct timeval timeout;

	// initialise the timeout structure
	timeout.tv_sec = 10; // ten second timeout
	timeout.tv_usec = 0;

	//Create byte array
	unsigned char send_bytes[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

	write(fd, send_bytes, 13);  //Send data
	ROS_ERROR("Wrote the bytes. \n");

	// do the select
	n = select(fd + 1, &rdfs, NULL, NULL, &timeout);

	// check if an error has occured
	if (n < 0) {
		ROS_ERROR("select failed\n");
	} else if (n == 0) {
		ROS_ERROR("Timeout!");
	} else {
		ROS_ERROR("\nBytes detected on the port!\n");
	}

	return 0;

} //query_modem

int main(int argc, char **argv) {
	ros::init(argc, argv, "hardware_communication");
	ros::NodeHandle n;

	// Open Port
	fd = open_port();
	configure_port(fd);
	ROS_INFO("Port Opened");
	tcflush(fd, TCIOFLUSH);

	// Establish Connection via 3 way handshake
	ROS_ERROR("1 way");
	while(strcmp(robotState.message, "START")) {
		robotState = receive_state();
	}

	return 1;

	ROS_ERROR("2 way");
	robotGoalPtr = &robotGoal;
	robotGoal.id = 1;
	memset(robotGoal.message,0,strlen(robotGoal.message));
	sprintf(robotGoal.message, "BEGIN");
	while(strcmp(robotState.message, "ACK")) {
		ROS_ERROR("Sending Begin");
		send_goal(robotGoalPtr);
		ros::Duration(0.5).sleep();
		ROS_ERROR("Receiving ACK");
		robotState = receive_state();
		print_robot_state(robotState);
	}
	ROS_ERROR("3 way");
	return 1;

	// Output
	fd_set serialfd;
	timeval t;
	t.tv_usec = 0;
	t.tv_sec = 1;

	// Output poll
	robotGoal.id = 1;
	sprintf(robotGoal.message, "Message 1");

	ros::Rate r(1);
	while (ros::ok()) {

		ROS_ERROR("Send Message %d", robotGoal.id);

		while (!strcmp(robotState.message, "ACK")) {
			// Send message
			send_goal(robotGoalPtr);

			// Wait for acknowledgment
			FD_ZERO(&serialfd);
			FD_SET(fd, &serialfd);
			t.tv_sec = 1;
			int retval = select(fd + 1, &serialfd, NULL, NULL, &t);
			if (retval == -1) {
				exit(4);
			} else if (retval == 0) {
				ROS_ERROR("Timeout");
			} else {
				robotState = receive_state();
			}
		}

		ROS_ERROR("Ack Message %s", robotState.message);
		robotGoal.id++;
		if(robotGoal.id > 10) break;
		r.sleep();
	}
}
