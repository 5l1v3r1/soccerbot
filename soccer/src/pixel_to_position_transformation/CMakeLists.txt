cmake_minimum_required(VERSION 2.8.3)
project(pixel_to_position_transformation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_msgs
  humanoid_league_msgs
  image_transport
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


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
#  INCLUDE_DIRS include
#  LIBRARIES pixel_to_position_transformation
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(POSE_ESTIMATION
	src/pose_estimation/CsvReader.cpp
	src/pose_estimation/CsvWriter.cpp
	src/pose_estimation/Mesh.cpp
	src/pose_estimation/Model.cpp
	src/pose_estimation/ModelRegistration.cpp
	src/pose_estimation/PnPProblem.cpp
	src/pose_estimation/RobustMatcher.cpp
	src/pose_estimation/Utils.cpp
)
add_library(POSE_ESTIMATION_LIBRARIES ${POSE_ESTIMATION} ${OpenCV_LIBRARIES})

## Calibrate Camera
add_executable(calibrate_camera src/calibrate_camera.cpp src/settings.cpp)
add_dependencies(calibrate_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(calibrate_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Ball Position Transformation
add_executable(ball_position_transformation src/ball_position_transformation.cpp)
add_dependencies(ball_position_transformation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ball_position_transformation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${POSE_ESTIMATION_LIBRARIES})

## Opponent Position Transformation
add_executable(opponent_position_transformation src/opponent_position_transformation.cpp)
add_dependencies(opponent_position_transformation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(opponent_position_transformation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${POSE_ESTIMATION_LIBRARIES})

## Goal Position Transformation
add_executable(goal_position_transformation src/goal_position_transformation.cpp)
add_dependencies(goal_position_transformation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(goal_position_transformation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${POSE_ESTIMATION_LIBRARIES})

## Line Position Transformation
add_executable(line_position_transformation src/line_position_transformation.cpp)
add_dependencies(line_position_transformation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(line_position_transformation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${POSE_ESTIMATION_LIBRARIES})

## Line Points Transformation
add_executable(line_points_transformation src/line_points_transformation.cpp)
add_dependencies(line_points_transformation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(line_points_transformation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${POSE_ESTIMATION_LIBRARIES})


#############
## Install ##
#############

#############
## Testing ##
#############
