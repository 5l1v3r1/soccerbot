cmake_minimum_required(VERSION 2.8)
project(UTRAFCBase CXX C)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(Rcssnet3d REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type selected, default to Release")
  set (CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

if(NOT CMAKE_CXX_FLAGS)
  set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-unused")
endif(NOT CMAKE_CXX_FLAGS)

include_directories(
${RCSSNET3D_INCLUDE_DIR}
)

set(SRCS
  main.cpp
  behaviours/naobehaviour.cpp
  behaviours/behaviour.cpp
)

add_executable(utracontrol 
${SRCS}
)

target_link_libraries(utracontrol
 ${RCSSNET3D_LIBRARY}
)

