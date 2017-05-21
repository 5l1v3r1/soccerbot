#ifndef BALL_SRC_BALL_HPP_
#define BALL_SRC_BALL_HPP_

#define PI 3.14159826

#define BALL_SIZE 3
#define BALL_DIAMETER   130
#define BALL_RADIUS    21  // 20.6
#define BALL_WEIGHT    130
#define BALL_COLOR    WHITE

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Transform.h>

class Ball {
private:
    int circumference();
    
    // Publishers and subscribers
    ros::NodeHandle n;
    ros::Publisher ball_pub;
    ros::Subscriber ball_sub;
    
    // The marker to draw
    visualization_msgs::Marker marker;
    
public:
    static const int radius = BALL_RADIUS; // Radius in millimeters

    Ball(ros::NodeHandle n);
    ~Ball();
    
    void draw();
    void update_location(geometry_msgs::Transform);
};

#endif /* BALL_SRC_BALL_HPP_ */
