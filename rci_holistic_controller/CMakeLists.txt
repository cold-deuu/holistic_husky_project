cmake_minimum_required(VERSION 3.0.2)
project(rci_holistic_controller)
add_compile_options(-std=c++17 -Ofast)

SET(CXX_DISABLE_WERROR TRUE)
SET(CMAKE_VERBOSE_MAKEFILE TRUE)
SET(PROJECT_USE_CMAKE_EXPORT TRUE)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(pinocchio REQUIRED)
find_package(eiquadprog REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  tf 
 )


find_package(catkin REQUIRED)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME} 
 CATKIN_DEPENDS 
)

include_directories(include )
ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)


# Main Library
SET(${PROJECT_NAME}_MATH_HEADERS
  include/rci_holistic_controller/math/math.hpp
  )

SET(${PROJECT_NAME}_TRAJECTORIES_HEADERS
  include/rci_holistic_controller/trajectory/SE3_cubic.hpp
  include/rci_holistic_controller/trajectory/joint_cubic.hpp
  )

SET(${PROJECT_NAME}_SOLVERS_HEADERS
  include/rci_holistic_controller/solver/qp_solver_mm.hpp

  )

SET(${PROJECT_NAME}_ROBOTS_HEADERS
  include/rci_holistic_controller/robot/robot.hpp
  )


SET(${PROJECT_NAME}_HEADERS
  ${${PROJECT_NAME}_MATH_HEADERS}
  ${${PROJECT_NAME}_TRAJECTORIES_HEADERS}
  ${${PROJECT_NAME}_SOLVERS_HEADERS}
  ${${PROJECT_NAME}_ROBOTS_HEADERS}  
  )

LIST(REMOVE_DUPLICATES ${PROJECT_NAME}_HEADERS)

SET(${PROJECT_NAME}_MATH_SOURCES
  src/math/math.cpp
  )


SET(${PROJECT_NAME}_TRAJECTORIES_SOURCES
  src/trajectory/SE3_cubic.cpp
  src/trajectory/joint_cubic.cpp
  )

SET(${PROJECT_NAME}_SOLVERS_SOURCES
  src/solver/qp_solver_mm.cpp
  )

SET(${PROJECT_NAME}_ROBOTS_SOURCES
  src/robot/robot.cpp
  )


SET(${PROJECT_NAME}_SOURCES
  ${${PROJECT_NAME}_MATH_SOURCES}
  ${${PROJECT_NAME}_ROBOTS_SOURCES}
  ${${PROJECT_NAME}_TRAJECTORIES_SOURCES}
  ${${PROJECT_NAME}_SOLVERS_SOURCES}
  )


SET(CMAKE_BUILD_TYPE "Release")

ADD_LIBRARY(${PROJECT_NAME} SHARED  ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include> )
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC eiquadprog::eiquadprog pinocchio::pinocchio)

