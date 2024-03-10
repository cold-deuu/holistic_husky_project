#include <iostream>
#include "holistic_action_manager/simulator/simulator.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <cmath>

using namespace holistic_controller::robot;
using namespace holistic_controller::math;
using namespace holistic_controller::trajectory;
using namespace holistic_controller::qp_solver;
using namespace Eigen;
using namespace pinocchio;
using namespace std;

//For Hololens
void holo_publish(Eigen::VectorXd &q_des);
ros::Publisher holo_pub_;

int main(int argc, char** argv)
{
    //Initial Setting
    ros::init(argc,argv,"husky_franka_server");
    ros::NodeHandle nh; 
    ros::Rate loop_rate(1000);
    
    //RobotWrapper
    string model_path, urdf_name;
    nh.getParam("/urdf_path", model_path);  
    nh.getParam("/urdf_name",urdf_name);
    vector<string> package_dirs;
    package_dirs.push_back(model_path);
    string urdfFileName = package_dirs[0] + urdf_name;
    RobotWrapper robot(urdfFileName, package_dirs, false);
    Model model = robot.model();
    Data data(robot.model());
    nq_ = robot.nq(); //nq_ = 12 : mobile x,y,theta / wheel : theta_r,theta_l / arm-joint : theta_1 ~ theta_7
    string ee_id = "panda_joint7";

    //controller
    ctrl_ = std::make_shared<RobotController::HuskyFrankaWrapper>(true, nh);
    ctrl_->initialize();


    //pub
    ros::Publisher mjc_pub = nh.advertise<mujoco_ros_msgs::JointSet>("/mujoco_ros/mujoco_ros_interface/joint_set", 5);
    mujoco_command_pub_ = nh.advertise<std_msgs::String>("/mujoco_ros/mujoco_ros_interface/sim_command_con2sim", 5);
    holo_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/holo_publisher", 5);

    //pub setting
    q_.setZero(nq_);
    v_.setZero(nq_);
    Eigen::VectorXd not_running_q(9);
    not_running_q.setZero();

    //mujoco
    mjc_msg_.position.resize(11);
    mjc_msg_.MODE = 0;
    mjc_msg_.time = time_;
    mjc_msg_.header.stamp = ros::Time::now();
    
    //sub
    ros::Subscriber jointState = nh.subscribe("mujoco_ros/mujoco_ros_interface/joint_states", 5, &JointStateCallback, ros::TransportHints().tcpNoDelay(true));    
    ros::Subscriber mujoco_time_sub = nh.subscribe("mujoco_ros/mujoco_ros_interface/sim_time", 1, &simTimeCallback, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber mujoco_command_sub = nh.subscribe("mujoco_ros/mujoco_ros_interface/sim_command_sim2con", 5, &simCommandCallback, ros::TransportHints().tcpNoDelay(true));
    
    holistic_action_server_ = std::make_unique<HolisticActionServer>("holistic_action_manager/holistic_control",nh,ctrl_);
    joint_posture_action_server_ = std::make_unique<JointPostureActionServer>("holistic_action_manager/joint_posture", nh, ctrl_);



    while(ros::ok()){
        ctrl_->compute_all_terms();
        holistic_action_server_->compute(ros::Time::now());
        joint_posture_action_server_->compute(ros::Time::now());
    

        if (!holistic_action_server_->isrunning() && !joint_posture_action_server_->isrunning())
        {
            ctrl_->static_ctrl(not_running_q);
        }
        
        else{
            not_running_q = ctrl_->get_pub().pub_q;
        }
        //pub
        mjc_msg_.time = time_; 
        mjc_msg_.header.stamp = ros::Time::now();
        mjc_msg_.position[0] = ctrl_->get_pub().pub_q(0);
        mjc_msg_.position[2] = ctrl_->get_pub().pub_q(0);
        mjc_msg_.position[1] = ctrl_->get_pub().pub_q(1);
        mjc_msg_.position[3] = ctrl_->get_pub().pub_q(1);

        for(int i=0; i<7; i++)
        {   
            mjc_msg_.position[i+4] = ctrl_->get_pub().pub_q(i+2);
        }
        mjc_pub.publish(mjc_msg_);

        ros::spinOnce();
        loop_rate.sleep();
        holo_publish(q_);
    }
    
    return 0;
}

void simTimeCallback(const std_msgs::Float32ConstPtr &msg){
    mujoco_time_ = msg->data;
    time_ = mujoco_time_;
}

void simCommandCallback(const std_msgs::StringConstPtr &msg){
    std::string buf;
    buf = msg->data;

    if (buf == "RESET")
    {
        std_msgs::String rst_msg_;
        rst_msg_.data = "RESET";
        mujoco_command_pub_.publish(rst_msg_);
        ROS_INFO_STREAM(rst_msg_);
    }

    if (buf == "INIT")
    {
        std_msgs::String rst_msg_;
        rst_msg_.data = "INIT";
        mujoco_command_pub_.publish(rst_msg_);
        mujoco_time_ = 0.0;
        ROS_INFO_STREAM(rst_msg_);
    }
}

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
    Eigen::VectorXd q_wheel(2), v_wheel(2);
    Eigen::VectorXd q_mobile(3), v_mobile(3);
    for (int i=0; i<2; i++){
        q_mobile(i) = msg->position[i];
        v_mobile(i) = msg->velocity[i];
        q_wheel(i) = msg->position[i+7];
        v_wheel(i) = msg->velocity[i+6];
    }

    q_mobile(2) = atan2(2.* (msg->position[5] * msg->position[4] + msg->position[6] * msg->position[3]), 1- 2.*(pow( msg->position[6], 2) + pow(msg->position[5], 2)));   
    v_mobile(2) = msg->velocity[5];

    Eigen::VectorXd q_arm(7), v_arm(7);
    for (int i=0; i< 7; i++){ 
        q_arm(i) = msg->position[i+11];
        v_arm(i) = msg->velocity[i+10];
    }
    q_.head(5) << q_mobile(0),q_mobile(1),q_mobile(2),q_wheel(0), q_wheel(1);
    q_.tail(7) =  q_arm;
    v_.head(5) << v_mobile(0),v_mobile(1),v_mobile(2),v_wheel(0),v_wheel(1);
    v_.tail(7) = v_arm;

    ctrl_->joint_update(q_,v_);
}
void holo_publish(Eigen::VectorXd &q_des){
    std_msgs::Float64MultiArray msg;
    for (int i=0;i<10;i++){
        if(i<3)
            msg.data.push_back(q_des(i));
        else
            msg.data.push_back(q_des(i+2));
    }
    holo_pub_.publish(msg);
}
