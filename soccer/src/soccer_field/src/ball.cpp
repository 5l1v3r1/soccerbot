#include "ball.hpp"

Ball::Ball(ros::NodeHandle n) {
    this->n = n;
    ball_pub = n.advertise<visualization_msgs::Marker>("ball", 1);
    
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "objects";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.lifetime = ros::Duration();
}

Ball::~Ball() {}

int Ball::circumference() {
    return 2 * PI*radius;
}

void update_location(geometry_msgs::Transform){
    
}

void Ball::draw() {
    ball_pub.publish(marker);
}