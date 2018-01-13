/*
 * packet.h
 *
 *  Created on: 2018-01-11
 *      Author: vuwij
 */

#ifndef HARDWARE_COMMUNICATION_INCLUDE_HARDWARE_COMMUNICATION_PACKET_H_
#define HARDWARE_COMMUNICATION_INCLUDE_HARDWARE_COMMUNICATION_PACKET_H_

#include "serial.h"

typedef struct packet {
	u_int8_t start_pattern;
	u_int8_t byte_count;
	u_int8_t data[PACKET_SIZE];
} Packet;

#endif /* HARDWARE_COMMUNICATION_INCLUDE_HARDWARE_COMMUNICATION_PACKET_H_ */
