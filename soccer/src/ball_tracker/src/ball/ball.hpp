/*
 * ball.hpp
 *
 *  Created on: Jan 24, 2017
 *      Author: soccer
 */

#ifndef BALL_SRC_BALL_HPP_
#define BALL_SRC_BALL_HPP_

#define PI 3.14159826

#define BALL_SIZE 3
#define BALL_DIAMETER			130
#define BALL_RADIUS				21 	// 20.6
#define BALL_WEIGHT				130
#define BALL_COLOR				WHITE


class Ball {
private:
	int circumference();

public:
	static const int radius = BALL_RADIUS;	// Radius in millimeters

	Ball();
	~Ball();

};


#endif /* BALL_SRC_BALL_HPP_ */
