cmake_minimum_required(VERSION 2.8)
project(UTAustinVillaBase CXX C)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/)

find_package(Threads REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Rcssnet3d REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type selected, default to Release")
  set (CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

if(NOT CMAKE_CXX_FLAGS)
  set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-unused")
endif(NOT CMAKE_CXX_FLAGS)

set(CORE_PATH utwalk)

include_directories(
${CORE_PATH}
${RCSSNET3D_INCLUDE_DIR}
)

set(CORE_FILES
  ${CORE_PATH}/MotionCore.cpp
  ${CORE_PATH}/Module.cpp  
  ${CORE_PATH}/common/NMatrix.cpp 
  ${CORE_PATH}/math/MVTools.cpp 
  ${CORE_PATH}/math/RotationMatrix.cpp 
  ${CORE_PATH}/memory/Lock.cpp 
  ${CORE_PATH}/memory/Logger.cpp   
  ${CORE_PATH}/memory/Memory.cpp  
  ${CORE_PATH}/memory/MemoryBlock.cpp  
  ${CORE_PATH}/memory/PrivateMemory.cpp  
  ${CORE_PATH}/memory/SharedMemory.cpp 
  ${CORE_PATH}/motion/MotionModule.cpp
  ${CORE_PATH}/motion/UTWalkEngine.cpp 
  ${CORE_PATH}/sensor/SensorModule.cpp  
  ${CORE_PATH}/sensor/InertialFilter.cpp 
  ${CORE_PATH}/kinematics/InverseKinematics.cpp 
  ${CORE_PATH}/kinematics/ForwardKinematics.cpp  
  ${CORE_PATH}/kinematics/KinematicsModule.cpp  
  ${CORE_PATH}/math/Geometry.cpp  
  ${CORE_PATH}/common/PIDController.cpp 
  ${CORE_PATH}/motion/WalkEngineParameters.cpp 
)

set(SRCS
  main.cpp 
  behaviors/behavior.cpp
  behaviors/naobehavior.cpp
  behaviors/checkfall.cpp 
  behaviors/kicking.cpp 
  behaviors/strategy.cpp
  behaviors/pkbehaviors.cpp
  behaviors/gazebobehavior.cpp
  servercomm/primitives.cpp
  parser/parser.cpp 
  math/hctmatrix.cpp
  math/vecposition.cpp
  math/Geometry.cpp
  worldmodel/worldmodel.cpp
  bodymodel/bodymodel.cpp
  particlefilter/PFLocalization.cpp
  particlefilter/Particle.cpp
  skills/skill.cpp 
  skills/curve3d.cpp
  ikfast/ikfast.cpp
  headers/headers.cpp
  audio/audio.cpp 
  rvdraw/rvdraw.cpp 
  ${CORE_FILES}
  kalman/BallKF.cpp
  kalman/PlayerKF.cpp 
  kalman/OrigKalmanFilter.cpp
  optimization/optimizationbehaviors.cpp
)

add_executable(agentspark 
${SRCS}
)

target_link_libraries(agentspark
${RCSSNET3D_LIBRARY}
${Boost_SYSTEM_LIBRARY}
dl
${CMAKE_THREAD_LIBS_INIT}
rt 
)

