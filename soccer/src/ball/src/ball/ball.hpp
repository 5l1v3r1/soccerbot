/*
 * ball.hpp
 *
 *  Created on: Jan 24, 2017
 *      Author: soccer
 */

#ifndef BALL_SRC_BALL_HPP_
#define BALL_SRC_BALL_HPP_

#define BALL_SIZE 3
#define PI 3.14159826

class Ball {
private:
	int circumference();

public:
	const int radius = 93;	// Radius in millimeters

	Ball();
	~Ball();

};


#endif /* BALL_SRC_BALL_HPP_ */
