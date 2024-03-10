cmake_minimum_required(VERSION 3.0.2)
project(holistic_action_manager)
add_compile_options(-std=c++17 -Ofast)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(pinocchio REQUIRED)
find_package(eiquadprog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rci_holistic_controller REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  mujoco_ros_msgs
  pluginlib
  tf 
  actionlib_msgs
  actionlib
  rci_holistic_controller
 )



find_package(catkin REQUIRED)


add_action_files(DIRECTORY action FILES 
  Holistic.action
  JointPosture.action
)

add_service_files(FILES se3_goal.srv unity.srv)

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs actionlib_msgs)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME} 
 CATKIN_DEPENDS 
 controller_interface
 pluginlib
 roscpp
 mujoco_ros_msgs
 sensor_msgs
 std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${rci_holistic_controller_INCLUDE_DIRS}
)
ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)

install(PROGRAMS
	
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


# Library
SET(${PROJECT_NAME}_HEADERS
  #Simulator
  include/holistic_action_manager/controller/controller.hpp
  include/holistic_action_manager/simulator/simulator.hpp

  #Server
  include/holistic_action_manager/server/holistic_server.hpp
  include/holistic_action_manager/server/action_server_base.hpp
  include/holistic_action_manager/server/joint_posture_action_server.hpp

  )

SET(${PROJECT_NAME}_SOURCES
  #Controller
  src/controller/controller.cpp
  
  #Server
  src/server/holistic_server.cpp
  src/server/joint_posture_action_server.cpp
)


SET(CMAKE_BUILD_TYPE "Release")

ADD_LIBRARY(${PROJECT_NAME} SHARED  ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include> )
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC ${rci_holistic_controller_LIBRARIES} ${pinocchio_LIBRARIES} eiquadprog::eiquadprog -lpthread)



#add_executable(${PROJECT_NAME}_pose src/non_holo.cpp)
#add_dependencies(${PROJECT_NAME}_pose ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}_pose ${PROJECT_NAME} eiquadprog::eiquadprog pinocchio::pinocchio)
#target_link_libraries(${PROJECT_NAME}_pose ${catkin_LIBRARIES})

#Test
add_executable(${PROJECT_NAME}_action src/simulator/simulator.cpp)
add_dependencies(${PROJECT_NAME}_action ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_action ${PROJECT_NAME} eiquadprog::eiquadprog pinocchio::pinocchio)
target_link_libraries(${PROJECT_NAME}_action ${catkin_LIBRARIES})


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
