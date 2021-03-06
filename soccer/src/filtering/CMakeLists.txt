cmake_minimum_required(VERSION 2.8.3)
project(filtering)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  gazebo_ros  
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(config/soccerbot.cfg)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  CATKIN_DEPENDS 
#    sensor_msgs
#    gazebo_ros 
#    roscpp
#    dynamic_reconfigure
#  DEPENDS 
#    gazebo
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Personal Model
add_executable(personal_model src/personal_model.cpp)
add_dependencies(personal_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(personal_model ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Team Updated Model
add_executable(team_updated_model src/team_updated_model.cpp)
add_dependencies(team_updated_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(team_updated_model ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############
