cmake_minimum_required(VERSION 3.0.2)
project(franka_mujoco)
add_compile_options(-std=c++17 -Ofast)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}" )

OPTION(EIGEN_RUNTIME_NO_MALLOC "If ON, it can assert in case of runtime allocation" ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  franka_hw
  franka_gripper
  geometry_msgs
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  rospy
  roslib
  tf
  mujoco_ros_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs actionlib_msgs sensor_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    franka_hw
    geometry_msgs
    hardware_interface
    pluginlib
    realtime_tools
    tf
    roscpp
    mujoco_ros_msgs
    message_runtime 
    std_msgs
  DEPENDS Franka
)



