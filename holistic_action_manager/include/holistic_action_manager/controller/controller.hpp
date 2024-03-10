#ifndef __husky_franka_ctrl__
#define __husky_franka_ctrl__

//Pinocchio Header
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp> 

//ROS Header
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Int16.h"
#include "tf/transform_datatypes.h"

//Holistic Controller
#include <rci_holistic_controller/robot/robot.hpp>
#include <rci_holistic_controller/trajectory/joint_cubic.hpp>
#include <rci_holistic_controller/trajectory/SE3_cubic.hpp>
#include <rci_holistic_controller/math/math.hpp>
#include <rci_holistic_controller/solver/qp_solver_mm.hpp>

using namespace std;
using namespace Eigen;
using namespace pinocchio;

using namespace holistic_controller::qp_solver;

typedef struct Pub_state {   
    Eigen::VectorXd pub_q;
    double pose_err;
} pub_state;   

namespace RobotController{
    class HuskyFrankaWrapper{
        public: 
            HuskyFrankaWrapper(const bool & issimulation, ros::NodeHandle & node);
            ~HuskyFrankaWrapper(){};

            void initialize();
            void compute_all_terms();

            void compute_arm_control(ros::Time ctime);
            void compute_holistic(ros::Time ctime);
            void init_holistic_compute(ros::Time stime, ros::Duration dur);
            void init_joint_compute(ros::Time stime, ros::Duration dur);
            
            void get_se3_task(SE3 &goal){
                m_wTep = goal;
            }

            void get_armjoint_task(Eigen::VectorXd &goal){
                // m_armjoint_goal = m_q;
                m_armjoint_goal = goal;
            }

            void joint_update(const Eigen::VectorXd& q, const Eigen::VectorXd& v);

            bool simulation(){
                return issimulation_;
            }
            void static_ctrl(Eigen::VectorXd & static_q);

            Pub_state & get_pub(){
                return m_pub;
            }

        private:
            bool issimulation_, mode_change_, franka_gripper_;
            std::string robot_node_;
            
            int con_num_;
            int arm_joint_;

            Pub_state m_pub;

            double time_;
            ros::Time stime_;
            int array_cnt_;
            int na_, nv_, nq_;
            bool isfinished_;

            std::shared_ptr<holistic_controller::robot::RobotWrapper> robot_;
            std::shared_ptr<holistic_controller::trajectory::JointCubicTrajectory> joint_traj_;
            std::shared_ptr<holistic_controller::trajectory::SE3CubicTrajectory> se3_traj_;
            pinocchio::Model model_;
            pinocchio::Data m_data,m_data_m;


            //solver
            Holistic_mm_qp m_mm_qp_solver;
            
            //init
            Eigen::VectorXd m_q_init,m_v_init;

            //goal
            SE3 m_wTep;
            Eigen::VectorXd m_armjoint_goal;
            ros::Duration m_dur;

            //Constraint
            Eigen::MatrixXd m_qlim;
            Eigen::VectorXd m_qdlim;

            //State
            Eigen::VectorXd m_q,m_v;
            Eigen::MatrixXd J_;
            Eigen::VectorXd m_v_current;
            SE3 m_pose_current;
            Eigen::VectorXd err;

            //Model
            Eigen::MatrixXd m_convert_mat;
            double r = 0.1651;
            double b = 0.2854;

            //for compute
            SE3 m_pose_cubic,m_pose_init;
            MatrixXd m_J_qp;
            VectorXd m_q_qp, m_base_q;
            
            //gain
            double m_Kp,m_Kv;


            string ee_id = "panda_joint7";
            ros::NodeHandle n_node_;
    };
}
#endif


