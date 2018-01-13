cmake_minimum_required(VERSION 2.8.3)
project(robot_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  humanoid_league_msgs
  nav_msgs
  message_generation
  visualization_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
    FILES
    WalkingPath.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)
################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_control
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Path Test
add_executable(path_test src/path_test.cpp)
add_dependencies(path_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_test ${catkin_LIBRARIES})

## Path Planning
add_executable(path_planning src/path_planning.cpp)
add_dependencies(path_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_planning ${catkin_LIBRARIES})

## Path Execution
add_executable(path_execution src/path_execution.cpp)
add_dependencies(path_execution ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_execution ${catkin_LIBRARIES})

#############
## Install ##
############

#############
## Testing ##
#############

