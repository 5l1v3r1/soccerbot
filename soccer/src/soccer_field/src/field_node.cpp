#include "ball.hpp"
#include "field.hpp"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "soccer_field");
  ros::NodeHandle n;
  
  ros::spin();
}