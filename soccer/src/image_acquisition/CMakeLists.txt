cmake_minimum_required(VERSION 2.8.3)
project(image_acquisition)

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
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
add_message_files(
   FILES
   ColorSpace.msg
   SoccerColorSpace.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)
################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
#   INCLUDE_DIRS include
#   CATKIN_DEPENDS roscpp std_msgs cv_bridge sensor_msgs geometry_msgs image_transport
#   DEPENDS system_lib
	CATKIN_DEPENDS 
	message_runtime
	std_msgs
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

## Image Acquisition
add_executable(image_acquisition src/image_acquisition.cpp)
add_dependencies(image_acquisition ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_acquisition ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Image Processing
add_executable(image_processing src/image_processing.cpp)
add_dependencies(image_processing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_processing ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Estimate Colorspace
add_executable(estimate_colorspace src/estimate_colorspace.cpp)
add_dependencies(estimate_colorspace ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(estimate_colorspace ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############
