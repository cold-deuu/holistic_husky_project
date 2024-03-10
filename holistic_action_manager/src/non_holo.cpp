#include <iostream>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <cmath>


// namespace plt = matplotlibcpp;

using namespace holistic_controller::robot;
using namespace holistic_controller::math;
using namespace holistic_controller::trajectory;
using namespace holistic_controller::qp_solver;
using namespace Eigen;
using namespace pinocchio;
using namespace std;

void loop(ros::Rate &loop_rate);
void mujoco_publish(Eigen::VectorXd &q_base_des, Eigen::VectorXd &q_arm_des);

ros::Publisher mjc_pub_;
int arm_joint_;

int main(int argc, char** argv)
{
    //for plot
    std::vector<double> t,roll,roll2,pit1,pit2,yaw1,yaw2,dx,dy,dz,dx2,dy2,dz2;

    //Initial Setting
    ros::init(argc,argv,"husky_panda_sim");
    ros::NodeHandle nh; 
    ros::Rate loop_rate(1000);
    
    //RobotWrapper
    const string ur_model_path = "/home/chan/hololens_pj/src/description_for_hololens";
    vector<string> package_dirs;
    package_dirs.push_back(ur_model_path);
    string urdfFileName = package_dirs[0] + "/husky_description/robots/husky_panda_other_pose.urdf";
    RobotWrapper robot(urdfFileName, package_dirs, false);
    const Model &model = robot.model();
    Data data(robot.model());
    Data data_m(robot.model());
    nq_ = robot.nq(); //nq_ = 12 : mobile x,y,theta / wheel : theta_r,theta_l / arm-joint : theta_1 ~ theta_7
    string ee_id = "panda_joint7";
    
    //pub
    mjc_pub_ = nh.advertise<mujoco_ros_msgs::JointSet>("/mujoco_ros/mujoco_ros_interface/joint_set", 5);
    mujoco_command_pub_ = nh.advertise<std_msgs::String>("/mujoco_ros/mujoco_ros_interface/sim_command_con2sim", 5);
    
    //sub
    ros::Subscriber jointState = nh.subscribe("mujoco_ros/mujoco_ros_interface/joint_states", 5, &JointStateCallback, ros::TransportHints().tcpNoDelay(true));    
    ros::Subscriber mujoco_time_sub = nh.subscribe("mujoco_ros/mujoco_ros_interface/sim_time", 1, &simTimeCallback, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber mujoco_command_sub = nh.subscribe("mujoco_ros/mujoco_ros_interface/sim_command_sim2con", 5, &simCommandCallback, ros::TransportHints().tcpNoDelay(true));

    //qp_solver
    Holistic_mm_qp_ mm_qp_solver;
    int con_num = 9;
    arm_joint_ = 7;

    //for pub
    Eigen::VectorXd qd_d(nq_),qd_(con_num),q_arm(arm_joint_);
    qd_.setZero();

    //joint limits  
    Eigen::MatrixXd qlim(2,con_num);
    Eigen::VectorXd qdlim(con_num);
    qlim << -10000,-10000,-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8793,
            10000,10000,2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8793;
    qdlim << 4,4, 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100;


    //initialize
    q_.setZero(nq_);
    v_.setZero(nq_);

    Eigen::VectorXd q_init_goal(nq_), v_init(nq_), q_init(nq_),base_q(2);
    q_init_goal.setZero();
    q_init_goal.tail(7)<<0, 0, 0, -M_PI/2, 0, M_PI/2, 0;
    v_init = v_;
    q_init = q_;
    J_.setZero(6,nq_);
    base_q.setZero();

    SE3 wMl;
    SE3 pose_cubic;

    //for non-holonomic
    double r = 0.1651;
    double b = 0.2854;
    Eigen::MatrixXd convert_mat(2,2);
    convert_mat<< -r/(2*b), r/(2*b),
                       r/2,     r/2;

    //for current pose
    Motion v_m_current;
    Eigen::VectorXd v_current(6);
    SE3 pose_current;
    Eigen::VectorXd err(6);

    //for Manipulability
    Eigen::VectorXd Jm(con_num-2);

    //for QP
    Eigen::MatrixXd J_qp(6,con_num);
    J_qp.setZero();
    Eigen::VectorXd q_qp(con_num);
    q_qp.setZero();

    Eigen::MatrixXd J1(6,nq_-3),J2(6,con_num-2), J_2(6,nq_), J_for_man(6,con_num-2);
    Eigen::VectorXd q1(con_num-2) ,q2(con_num-2),q_m(nq_);
    J_2.setZero();

    //mjc_msg
    mjc_msg_.position.resize(11);
    mjc_msg_.MODE = 0;
    mjc_msg_.time = time_;
    mjc_msg_.header.stamp = ros::Time::now();

    //pose_init
    ros::Time init_stime = ros::Time::now();
    ros::Duration init_dur = ros::Duration(2.0);

    while((ros::Time::now()-init_stime).toSec()<init_dur.toSec()+0.5){

        Eigen::VectorXd init_q_cubic = holistic_controller::trajectory::q_cubic(init_stime,q_init,v_init,q_init_goal,init_dur,ros::Time::now());
        base_q = init_q_cubic.segment(3,2);
        q_arm = init_q_cubic.tail(arm_joint_);

        mujoco_publish(base_q, q_arm);
        robot.computeAllTerms(data,q_,v_);
        loop(loop_rate);
    }
    robot.jacobianWorld(data,model.getJointId("panda_joint7"),J_);
    double manip = manipulability(J_);

    SE3 pose_init = robot.position(data,model.getJointId(ee_id));
    
    Eigen::MatrixXd rot_base(2,2),rot_arm_base(2,2);
    double rad;
    rot_base = robot.position(data,model.getFrameId("base_link")).rotation().topLeftCorner(2,2);
    rot_arm_base = robot.position(data,model.getJointId("panda_joint1")).rotation().topRightCorner(2,2);

    rad = atan2((rot_base.inverse() * rot_arm_base)(1,0),(rot_base.inverse()*rot_arm_base)(0,0));
    std::cout << "AAAA "<<rad << std::endl;
    //goal setting
    pinocchio::SE3 wTep;

    // pose_init = robot.position(data,model.getFrameId("base_link"));
    wTep.translation() = pose_init.translation();
    // wTep.rotation().setIdentity();
    // wTep.translation()<<3.0,4.0,0.4;

    // TEST - OKAY
    wTep.rotation() = pose_init.rotation();
    wTep.translation()(0) += 3.0;
    wTep.translation()(1) += 2.0;
    wTep.translation()(2) -= 0.1;

    // wTep.translation()(0) -= 4.0;
    // wTep.translation()(2) -= 0.25;
    // wTep.rotation() = pose_init.rotation();

    //iteration settings
    int k = 1;
    double pose_gain=15;
    double vel_gain =0;
    double et;
    double m1 =0;
    q_qp.tail(7) = q_.tail(7);

    Eigen::VectorXd q_for_m(nq_),qm1(7),qm2(7);
    Eigen::MatrixXd J_for_m(6,nq_),Jm1(6,7),Jm2(6,7);
    q_for_m.setZero();
    qm1.setZero();
    qm2.setZero();
    J_for_m.setZero();
    Jm1.setZero();
    Jm2.setZero();

    ros::Time stime = ros::Time::now();
    ros::Duration duration = ros::Duration(10.0);

    // while((ros::Time::now()-stime).toSec() < duration.toSec() + 0.5)
    while(ros::ok())
    {   
        //init iteration
        robot.computeAllTerms(data,q_,v_);
        robot.jacobianWorld(data, model.getJointId(ee_id), J_);

        double phi = atan2(robot.framePosition(data,model.getFrameId("base_link")).rotation()(1,0),robot.framePosition(data,model.getFrameId("base_link")).rotation()(0,0));
        q_qp.head(2).setZero();
        J_qp.topRightCorner(6,7) = J_.topRightCorner(6,7);
        
        J_qp.topLeftCorner(6,2) << 0,cos(q_(2)),
                                   0,sin(q_(2)),
                                   0,    0,
                                   0,    0,
                                   0,    0,
                                   1,    0;

        Jm = holistic_controller::math::getJacobM(robot,q_,v_,ee_id,arm_joint_);

        // Jm = holistic_controller::math::getJacobM(robot,q_qp,v_,ee_id,arm_joint_);
        // cout<<Jm.transpose()<<endl;
        //get trajectory
        pose_cubic = holistic_controller::trajectory::se3_cubic(stime,pose_init,wTep,duration,ros::Time::now());
        SE3 pose_current_base = robot.position(data,model.getJointId("panda_joint1"));
        SE3 base_se3 = robot.framePosition(data,model.getFrameId("base_link"));
        pose_current = robot.position(data,model.getJointId(ee_id));

        Eigen::VectorXd tran_bTe(3);
        tran_bTe = (base_se3.rotation().inverse())*(pose_current.translation() - pose_current_base.translation());
        Eigen::VectorXd base_to_goal(3);
        base_to_goal = base_se3.rotation().inverse() * (wTep.translation() - pose_current_base.translation());
        
        double theta = atan2(base_to_goal(1),base_to_goal(0));


        if(k==1 || et>0.5)
            et = abs(wTep.translation()(0)-pose_current.translation()(0)) + abs(wTep.translation()(1)-pose_current.translation()(1)) +abs(wTep.translation()(2)-pose_current.translation()(2));

        // et = abs(wTep.translation()(0)-pose_current.translation()(0)) + abs(wTep.translation()(1)-pose_current.translation()(1)) +abs(wTep.translation()(2)-pose_current.translation()(2));

            // ROS_INFO_STREAM(head_angle);

        //qp_solve
        err = holistic_controller::math::get_error_6d(pose_current,pose_cubic);

        if(k%100==0)
            ROS_INFO_STREAM(err.transpose());

        wMl.rotation(pose_current.rotation());
        v_m_current = robot.velocity(data,model.getJointId(ee_id));
        v_current = v_m_current.toVector(); 

        mm_qp_solver.qp_setting(err,v_current,con_num,J_qp,Jm,qlim,qdlim,q_qp,tran_bTe);
        mm_qp_solver.gain_setting(pose_gain,vel_gain,et,theta);
        qd_ = mm_qp_solver.solve();
        // qd_ = holistic_controller::math::pseudoinv(J_qp) * (6*err);

        //iteration
        q_qp.tail(7) = q_qp.tail(7) + qd_.tail(7)*0.001;
        base_q += (convert_mat.inverse() *qd_.head(2))*0.001;
        q_arm = q_qp.tail(7);

        //pub
        mujoco_publish(base_q, q_arm);        
        loop(loop_rate);
        k+=1;
    }

    ROS_INFO_STREAM("DESIRED");
    ROS_INFO_STREAM(wTep);
    ROS_INFO_STREAM("RESULT");
    ROS_INFO_STREAM(robot.position(data,model.getJointId(ee_id)));
    ROS_ERROR_STREAM("-======================");

    
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
    }

    if (buf == "INIT")
    {
        std_msgs::String rst_msg_;
        rst_msg_.data = "INIT";
        
        mujoco_time_ = 0.0;
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
}

void loop(ros::Rate &loop_rate){
    loop_rate.sleep();
    ros::spinOnce();
}

void mujoco_publish(Eigen::VectorXd &q_base_des, Eigen::VectorXd &q_arm_des){

        mjc_msg_.time = time_; 
        mjc_msg_.header.stamp = ros::Time::now();
        
        mjc_msg_.position[0] = q_base_des(0);
        mjc_msg_.position[2] = q_base_des(0);
        mjc_msg_.position[1] = q_base_des(1);
        mjc_msg_.position[3] = q_base_des(1);

        for(int i=0; i<7; i++)
        {   
            mjc_msg_.position[i+4] = q_arm_des(i);
        }

        mjc_pub_.publish(mjc_msg_);

}