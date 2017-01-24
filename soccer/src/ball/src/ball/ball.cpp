/*
 * ball.cpp
 *
 *  Created on: Jan 24, 2017
 *      Author: soccer
 */

#include "ball.hpp"

Ball::Ball() {

}

Ball::~Ball() {

}

int Ball::circumference() {
	return 2*PI*radius;
}

