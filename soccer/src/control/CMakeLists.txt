cmake_minimum_required(VERSION 2.8)
project(UTRAFCBase CXX C)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type selected, default to Release")
  set (CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

if(NOT CMAKE_CXX_FLAGS)
  set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-unused")
endif(NOT CMAKE_CXX_FLAGS)


set(SRCS
  main.cpp 
)

add_executable(agentspark 
${SRCS}
)

target_link_libraries(agentspark
 
)

