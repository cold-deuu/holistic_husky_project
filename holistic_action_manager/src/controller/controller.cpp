#include "holistic_action_manager/controller/controller.hpp"
#include <iostream>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <cmath>
#include <vector>

using namespace holistic_controller::robot;
using namespace holistic_controller::math;
using namespace holistic_controller::trajectory;
using namespace holistic_controller::qp_solver;
using namespace Eigen;
using namespace pinocchio;
using namespace std;

namespace RobotController{
    HuskyFrankaWrapper::HuskyFrankaWrapper(const bool & issimulation, ros::NodeHandle & node)
    : issimulation_(issimulation), n_node_(node)
    {
        m_armjoint_goal.resize(7);
        time_ = 0.;
        m_Kp = 6;
        m_Kv = 0;
    }

    void HuskyFrankaWrapper::initialize(){
        string model_path, urdf_name;
        n_node_.getParam("/urdf_path",model_path); 
        n_node_.getParam("/urdf_name",urdf_name);

        vector<string> package_dirs;
        package_dirs.push_back(model_path);
        string urdfFileName = package_dirs[0] + urdf_name;

        robot_ = std::make_shared<RobotWrapper>(urdfFileName, package_dirs, false);        
        model_ = robot_->model();

        Data data(robot_->model());
        m_data = data;

        na_ = robot_->na();
        nv_ = robot_->nv();
        nq_ = robot_->nq();

        //qp_solver
        con_num_ = 9;
        arm_joint_ = 7;

        //joint limits  
        m_qlim.resize(2,con_num_);
        m_qdlim.resize(con_num_);
        m_qlim << -10000,-10000,-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8793,
                10000,10000,2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8793;
        m_qdlim << 4,4, 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100;


        //Trajectory Pointer
        joint_traj_ = std::make_shared<JointCubicTrajectory>("Cubic_Trajectory", 7);
        se3_traj_ = std::make_shared<SE3CubicTrajectory>("SE3_Trajectory");

        //initialize
        m_q.setZero(nq_);
        m_v.setZero(nq_);
        J_.setZero(6,nq_);
        
        m_q_init.setZero(nq_);
        m_v_init.setZero(nq_);


        //for non-holonomic
        m_convert_mat.resize(2,2);
        m_convert_mat<< -r/(2*b), r/(2*b),
                         r/2,     r/2;

        //for QP
        m_J_qp.setZero(6,con_num_);
        m_q_qp.setZero(con_num_);
        m_base_q.setZero(2); 
    }

    void HuskyFrankaWrapper::init_joint_compute(ros::Time stime, ros::Duration dur){
        m_v_init.setZero();
        cout<<m_armjoint_goal<<endl;
        joint_traj_ -> SetStartTime(stime);
        joint_traj_ -> SetDuration(dur);
        joint_traj_ -> SetInitSample(m_q.tail(7));
        joint_traj_ -> SetGoalSample(m_armjoint_goal.tail(7));

    }

    void HuskyFrankaWrapper::compute_arm_control(ros::Time ctime){
        
        joint_traj_ -> SetCurrentTime(ctime);
        Eigen::VectorXd cubic_traj = joint_traj_ -> computeNext();    
        m_pub.pub_q.tail(7) = cubic_traj;
    }

    void HuskyFrankaWrapper::init_holistic_compute(ros::Time stime, ros::Duration dur){
        m_pose_init = robot_->position(m_data,model_.getJointId(ee_id));
        se3_traj_ -> SetStartTime(stime);
        se3_traj_ -> SetDuration(dur);
        se3_traj_ -> SetInitSample(m_pose_init);
        se3_traj_ -> SetGoalSample(m_wTep);
    }

    void HuskyFrankaWrapper::compute_holistic(ros::Time ctime)
    {
        //for Manipulability
        Eigen::VectorXd Jm(con_num_-2);

        //Jacob .... Change?
        robot_->jacobianWorld(m_data, model_.getJointId(ee_id), J_);
        m_q_qp.head(2).setZero();
        m_J_qp.topRightCorner(6,7) = J_.topRightCorner(6,7);

        m_J_qp.topLeftCorner(6,2) << 0,cos(m_q(2)),
                0,sin(m_q(2)),
                0,    0,
                0,    0,
                0,    0,
                1,    0;

        Jm = holistic_controller::math::getJacobM(*robot_,m_q,m_v,ee_id,arm_joint_);

        //get trajectory
        se3_traj_ -> SetCurrentTime(ctime);
        m_pose_cubic = se3_traj_ -> computeNext();

        SE3 pose_current_base = robot_->position(m_data,model_.getJointId("panda_joint1"));
        SE3 base_se3 = robot_->framePosition(m_data,model_.getFrameId("base_link"));
        m_pose_current = robot_->position(m_data,model_.getJointId(ee_id));

        Eigen::VectorXd tran_bTe(3);
        tran_bTe = (base_se3.rotation().inverse())*(m_pose_current.translation() - pose_current_base.translation());

        double et = abs(m_wTep.translation()(0)-m_pose_current.translation()(0)) + abs(m_wTep.translation()(1)-m_pose_current.translation()(1)) +abs(m_wTep.translation()(2)-m_pose_current.translation()(2));


        //qp_solve
        err = holistic_controller::math::get_error_6d(m_pose_current,m_pose_cubic);

        Motion v_m_current;
        v_m_current = robot_->velocity(m_data,model_.getJointId(ee_id));
        m_v_current = v_m_current.toVector();

        m_mm_qp_solver.qp_setting(err,m_v_current,con_num_,m_J_qp,Jm,m_qlim,m_qdlim,m_q_qp,tran_bTe);
        m_mm_qp_solver.gain_setting(m_Kp,m_Kv,et);
        
        Eigen::VectorXd qd(con_num_);
        qd = m_mm_qp_solver.solve();

        //iteration
        m_q_qp.tail(7) = m_q_qp.tail(7) + qd.tail(7)*0.001;
        m_base_q += m_convert_mat.inverse() *(qd.head(2)*0.001);

        Eigen::VectorXd q_des(9);
        q_des.head(2) = m_base_q;
        q_des.tail(7) = m_q_qp.tail(7);

        m_pub.pub_q.head(2) = m_base_q;
        m_pub.pub_q.tail(7) = m_q_qp.tail(7);
        m_pub.pose_err = et;


    }


    void HuskyFrankaWrapper::static_ctrl(Eigen::VectorXd & static_q){
        m_pub.pub_q = static_q;
        m_q_qp.tail(7) = static_q.tail(7);
        m_q_init.tail(7) = static_q.tail(7);
    }


    void HuskyFrankaWrapper::joint_update(const Eigen::VectorXd& q, const Eigen::VectorXd& v){        
        m_q = q;
        m_v = v;
    }

    void HuskyFrankaWrapper::compute_all_terms(){
        robot_->computeAllTerms(m_data, m_q, m_v);
    }

}