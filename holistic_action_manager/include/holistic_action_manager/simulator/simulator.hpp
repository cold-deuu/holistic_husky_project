//Holistic Controller
#include <rci_holistic_controller/robot/robot.hpp>
#include <rci_holistic_controller/math/math.hpp>

#include "holistic_action_manager/controller/controller.hpp"

//Mujoco MSG Header
#include "mujoco_ros_msgs/JointSet.h"
#include "mujoco_ros_msgs/SensorState.h"

#include <Eigen/QR>    
#include <Eigen/Core>
#include <ros/ros.h>
#include "eiquadprog/eiquadprog.hpp"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <stdio.h>
#include <tuple>
#include <string>
#include <vector>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"


//SYSTEM Header
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>

//HuskyFrankaAction
#include <holistic_action_manager/server/holistic_server.hpp>
#include <holistic_action_manager/server/joint_posture_action_server.hpp>



using namespace Eigen;

std::shared_ptr<RobotController::HuskyFrankaWrapper> ctrl_;

//pub
ros::Publisher mujoco_command_pub_;
ros::Publisher gui_joint_pub_;


//msg
mujoco_ros_msgs::JointSet mjc_msg_;

//varialbes - const
double mujoco_time_, time_;
int nq_;
string control_type_;


//Action Server
std::unique_ptr<HolisticActionServer> holistic_action_server_;
std::unique_ptr<JointPostureActionServer> joint_posture_action_server_;

//globla_variable
Eigen::VectorXd q_,v_;     
// Eigen::MatrixXd J_;
// pinocchio::SE3 wTep_;
// double dur_;

//Callback
void simTimeCallback(const std_msgs::Float32ConstPtr &msg);
void simCommandCallback(const std_msgs::StringConstPtr &msg);
void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

//for service
void holo_publish(Eigen::VectorXd &q_des);
