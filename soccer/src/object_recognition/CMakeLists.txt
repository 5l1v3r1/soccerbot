cmake_minimum_required(VERSION 2.8.3)
project(object_recognition)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  image_transport
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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
#  CATKIN_DEPENDS roscpp std_msgs
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

## Ball ROI
add_executable(ball_ROI src/ball_ROI.cpp)
add_dependencies(ball_ROI ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ball_ROI ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Ball Detection
add_executable(ball_detection src/ball_detection.cpp)
add_dependencies(ball_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ball_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Ball Candidate Chooser
add_executable(ball_candidate_chooser src/ball_candidate_chooser.cpp)
add_dependencies(ball_candidate_chooser ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ball_candidate_chooser ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Obstacle ROI
add_executable(obstacle_ROI src/obstacle_ROI.cpp)
add_dependencies(obstacle_ROI ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_ROI ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Obstacle Detection
add_executable(obstacle_detection src/obstacle_detection.cpp)
add_dependencies(obstacle_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Goal ROI
add_executable(goal_ROI src/goal_ROI.cpp)
add_dependencies(goal_ROI ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(goal_ROI ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Post Detection
add_executable(post_detection src/post_detection.cpp)
add_dependencies(post_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(post_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Goal Detection
add_executable(goal_detection src/goal_detection.cpp)
add_dependencies(goal_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(goal_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Line ROI
add_executable(line_ROI src/line_ROI.cpp)
add_dependencies(line_ROI ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(line_ROI ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Line Detection
add_executable(line_detection src/line_detection.cpp)
add_dependencies(line_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(line_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Visual Compass
add_executable(visual_compass src/visual_compass.cpp)
add_dependencies(visual_compass ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(visual_compass ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Visual Odometry
add_executable(visual_odom src/visual_odom.cpp)
add_dependencies(visual_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(visual_odom ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


#############
## Install ##
#############

#############
## Testing ##
#############

