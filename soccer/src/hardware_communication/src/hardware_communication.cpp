#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <termio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>
#include "serial.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <hardware_communication/RobotGoal.h>
#include <hardware_communication/RobotState.h>

using namespace std;
using namespace ros;
ros::NodeHandle* nh;
int fd;
hardware_communication::RobotState successcall;
RobotState robotState;
RobotGoal robotGoal;
RobotGoal *robotGoalPtr;

ros::Publisher hardware_publisher;
ros::Subscriber hardware_subscriber;

enum MoveType {
	turnLeft,
	turnRight,
	moveForward,
	success,
	failure,
	kick,
	idle,
	waiting
};

int open_port(void) {
	int fd; // file description for the serial port

	fd = open("/dev/ttyUSB0",
			O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK | O_SYNC);

	if (fd == -1)
		fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	if (fd == -1)
		fd = open("/dev/ttyUSB2", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	if (fd == -1) {
		cout << "open_port: Unable to open /dev/ttyACM0. \n" << endl;
		exit(1);
	}

	fcntl(fd, F_SETFL, 0);
	cout << "Opened Port" << endl;

	return (fd);
} //open_port

int configure_port(int fd) {
	struct termios port_settings;     // structure to store the port settings in

	cfsetispeed(&port_settings, B115200);    // set baud rates
	cfsetospeed(&port_settings, B115200);

	port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, data bits
	port_settings.c_cflag &= ~CSTOPB;
	port_settings.c_cflag &= ~CSIZE;
	port_settings.c_cflag |= CS8;

	port_settings.c_cflag |= (CLOCAL | CREAD);
	port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	port_settings.c_oflag &= ~OPOST;
	port_settings.c_cc[VMIN] = 0;
	port_settings.c_cc[VTIME] = 10;

	tcsetattr(fd, TCSANOW, &port_settings);    // apply the settings to the port
//	tcsetattr(fd,TCSAFLUSH,&port_settings);
//	fcntl(fd, F_SETFL, O_NONBLOCK);       // make the reads non-blocking

	cout << "Configured Port" << endl;
	return (fd);

} //configure_port

void establishConnection () {
	// Open Port
	fd = open_port();
	configure_port(fd);
	tcflush(fd, TCIOFLUSH);

	// Clean up
	memset(robotGoal.message, 0, strlen(robotGoal.message));
	memset(robotState.message, 0, strlen(robotState.message));

	// Establish Connection via 3 way handshake
	cout << "1 way" << endl;

	while (strcmp(robotState.message, "START")) {
		tcflush(fd, TCIFLUSH);
		robotState = receive_state();
		cout << robotState.id << " " << robotState.message << endl;
	}

	cout << "2 way" << endl;

	// Creating RobotGoal
	robotGoalPtr = &robotGoal;
	robotGoal.id = 1;
	memset(robotGoal.message, 0, strlen(robotGoal.message));
	sprintf(robotGoal.message, "BEGIN");

	while (strcmp(robotState.message, "ACK")) {
		tcflush(fd, TCIFLUSH);
		send_goal(robotGoalPtr);
		robotState = receive_state();
		cout << robotState.id << " " << robotState.message << endl;
	}
	cout << "3 way" << endl;
}

void messageLoop(ros::Publisher hardware_publisher) {
	memset(robotState.message, 0, strlen(robotState.message));
	// Send Goal
	while (strcmp(robotState.message, "ACK") || robotGoal.id != robotState.id) {
		tcflush(fd, TCIOFLUSH);
		send_goal(robotGoalPtr);
		cout << "Sending Message " << robotGoal.id << endl;

		usleep(200000);
		robotState = receive_state();
		cout << "Receiving Message " << robotState.id << endl;

		if(strcmp(robotState.message, "Done")){
			successcall.message = MoveType::success;
			hardware_publisher.publish(successcall);
			break;
		};
	}

	cout << robotState.id << " " << robotState.message << endl;
	robotGoal.id++;
}

void send_message(const hardware_communication::RobotStateConstPtr& msg){
	ROS_INFO("That's me");
	switch(msg->message){
	case turnLeft:
		ROS_INFO("turnLeft");
		strcpy(robotGoal.message,  "turnLeft");
		break;
	case turnRight:
		ROS_INFO("turnRight");
		strcpy(robotGoal.message,  "turnRight");
		break;
	case moveForward:
		ROS_INFO("forward");
		strcpy(robotGoal.message,  "forward");
		break;
	case success:
		ROS_INFO("success");
		strcpy(robotGoal.message,  "success");
		break;
	case failure:
		ROS_INFO("failure");
		strcpy(robotGoal.message,  "failure");
		break;
	case kick:
		ROS_INFO("kick");
		strcpy(robotGoal.message,  "kick");
		break;
	case idle:
		ROS_INFO("idle");
		strcpy(robotGoal.message,  "idle");
		break;
	default:
	    break;
	}
};

int main(int argc, char **argv) {

	//establishConnection();
	ros::init(argc, argv, "Hardware Communication");
	ros::NodeHandle n;
	nh = &n;
	hardware_subscriber = n.subscribe("/robot_control/execution", 1, send_message);
	hardware_publisher = n.advertise<hardware_communication::RobotState>("/hardware_communication/RobotState", 1);
	ros::Rate r(2);
	while(ros::ok()) {
		strcpy(robotGoal.message, "waiting");
		successcall.message = MoveType::waiting;
		hardware_publisher.publish(successcall);
		ROS_INFO("OK");
		messageLoop(hardware_publisher);
		ROS_INFO("Here");
		if(!strcmp(robotState.message, "Done")){
			strcpy(robotGoal.message, "waiting");
			successcall.message = MoveType::waiting;
			hardware_publisher.publish(successcall);
		};
		ros::spinOnce();
		r.sleep();
	}
}
