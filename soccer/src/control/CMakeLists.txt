cmake_minimum_required(VERSION 2.8)
project(UTRAFCBase CXX C)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(Boost COMPONENTS system REQUIRED)
find_package(Rcssnet3d REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type selected, default to Release")
  set (CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

if(NOT CMAKE_CXX_FLAGS)
  set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-unused")
endif(NOT CMAKE_CXX_FLAGS)

#set(RCSSNET3D_INCLUDE_DIR "/usr/include/simspark")
#set(RCSSNET3D_LIBRARY "../../../soccerbot/simspark/rcssserver3d")

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
    behaviours/behaviour.cpp
    behaviours/naobehaviour.cpp
    optimization/Optimizer.cpp
    math/hctmatrix.cpp
    math/vecposition.cpp
    math/Geometry.cpp
    models/worldmodel/worldmodel.cpp
    models/worldmodel/parser.cpp
    models/bodymodel/bodymodel.cpp
    skills/skill.cpp
    skills/curve3d.cpp
    headers/headers.cpp
)

add_executable(utracontrol 
${SRCS}
)

target_link_libraries(utracontrol
 ${RCSSNET3D_LIBRARY}
 ${Boost_SYSTEM_LIBRARY}
)

