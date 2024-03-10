#include "rci_holistic_controller/solver/qp_solver_mm.hpp"
#include "eiquadprog/eiquadprog.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <ros/ros.h>
#include <cmath>
#include <fstream>
#include <vector>
#include <algorithm>
using namespace Eigen;
using namespace std;

# define M_PI           3.14159265358979323846


namespace holistic_controller
{
    namespace qp_solver{
        
        Holistic_mm_qp::Holistic_mm_qp()
        {
            m_err_6d.resize(6);
            m_v_current.resize(6);
            m_trans_bTe.resize(3);
            m_slack.setZero(6);
        }


        void Holistic_mm_qp::qp_setting(Eigen::VectorXd &error_6d,Eigen::VectorXd &v_current, int &control_num,Eigen::MatrixXd &jacob, Eigen::VectorXd &jacobm, Eigen::MatrixXd &qlimit,Eigen::VectorXd &qdlimit, Eigen::VectorXd &current_q, Eigen::VectorXd &trans_bTe){
            m_n = control_num;
            m_Aeq.resize(6,m_n+6);
            m_Beq.resize(6);
            m_Q.resize(m_n+6,m_n+6);
            m_C.resize(m_n+6);
            m_Aineq.resize(3*m_n+12,m_n+6);
            m_Bineq.resize(3*m_n+12);
            m_x.resize(m_n+6);
            m_activeSet.resize(3*m_n + 18);
            m_Aeq.setZero();
            m_Beq.setZero();
            m_Q.setZero();
            m_C.setZero();
            m_Aineq.setZero();
            m_Bineq.setZero();
            m_x.setZero();

            m_err_6d.resize(6);
            m_J.resize(6,m_n);
            m_Jm.resize(m_n);
            m_qlim.resize(2,m_n);
            m_qdlim.resize(m_n);
            m_q.resize(m_n);
            
            m_trans_bTe = trans_bTe;
            m_err_6d = error_6d;
            m_v_current = v_current;
            m_J = jacob;
            m_Jm = jacobm;
            m_qlim = qlimit;
            m_qdlim = qdlimit;
            m_q = current_q;
        }

        void Holistic_mm_qp::gain_setting(double &position_gain, double &velocity_gain, double &et)
        {
            m_Kp = position_gain;
            m_Kd = velocity_gain;
            m_et = et;
        }

        void Holistic_mm_qp::set_Q(){
            //Slack
            m_Q.resize(m_n+6,m_n+6);
            m_Q.setIdentity();
            
            //Original
            m_Q.topLeftCorner(m_n,m_n) = m_Q.topLeftCorner(m_n,m_n)*1.5; 
            m_Q.topLeftCorner(2,2).setIdentity();
            m_Q.topLeftCorner(2,2) *= 1/m_et;
            m_Q.bottomRightCorner(6,6) *= 3/m_et;
        }

        void Holistic_mm_qp::set_C(){
            //Original
            double ke = 2; //original 
            m_C.resize(m_n+6);
            m_C.setZero();
            m_C.segment(2,m_n-2) = -m_Jm*1;
            m_C(0) = -ke * (atan2(m_trans_bTe(1),m_trans_bTe(0)));
        }

        void Holistic_mm_qp::set_eq(){
            Eigen::VectorXd v_star(6);
            v_star.setZero();
            m_Aeq.resize(6, m_n+6);
            m_Beq.resize(6);
            m_Aeq.setZero();
            m_Beq.setZero();

            v_star.head(3) = m_Kp * m_err_6d.head(3) - m_Kd * m_v_current.head(3);
            v_star.tail(3) = m_Kp/3 * m_err_6d.tail(3);
            m_Aeq.topLeftCorner(6,m_n) = m_J;
            m_Aeq.topRightCorner(6,6).setIdentity();

            m_Beq = -v_star+m_slack;

        }

        void Holistic_mm_qp::set_ineq(){
            double pi = 0.9;
            double ps = 0.1;
            double gain =1;
            

            m_Aineq.resize(3*m_n+12,m_n+6);
            m_Bineq.resize(3*m_n+12);
            m_Aineq.setZero();
            m_Bineq.setZero();

            for (int i=0; i<m_n; i++)
            {
                if(i<2){
                    m_Aineq(i,i) = 0;
                    m_Bineq(i) = 0;
                }

                else if(m_q(i) - m_qlim(0,i)<=m_qlim(1,i) - m_q(i)){
                    m_Bineq(i) = gain * (((m_q(i) - m_qlim(0,i))-ps)/(pi-ps));
                    m_Aineq(i,i) = -1;
                }
                else{
                    m_Bineq(i) = gain * (((m_qlim(1,i) - m_q(i))-ps)/(pi-ps));
                    m_Aineq(i,i) = 1;
                }
                
            }

            Eigen::MatrixXd A_lb,A_ub;
            A_lb.resize(m_n+6,m_n+6);
            A_ub.resize(m_n+6,m_n+6);
            A_lb.setIdentity();
            A_ub.setIdentity();
            Eigen::VectorXd B_lb,B_ub;
            B_lb.resize(m_n+6);
            B_ub.resize(m_n+6);

            B_lb.head(m_n) = m_qdlim* -1;
            B_ub.head(m_n) = m_qdlim;
            B_lb.tail(6) << -10,-10,-10,-10,-10,-10;
            B_ub.tail(6) << 10, 10, 10, 10, 10, 10;

            m_Aineq.block(m_n,0,m_n+6,m_n+6) = A_ub*-1;
            m_Aineq.block(2*m_n+6,0,m_n+6,m_n+6) = A_lb;
            m_Bineq.segment(m_n,m_n+6) = B_ub;
            m_Bineq.tail(m_n+6) = B_lb *-1;
        }

        Eigen::VectorXd Holistic_mm_qp::solve(){
            Holistic_mm_qp::set_Q();
            Holistic_mm_qp::set_C();
            Holistic_mm_qp::set_eq();
            Holistic_mm_qp::set_ineq();
            
            double out = eiquadprog::solvers:: 
                         solve_quadprog(m_Q, m_C, m_Aeq.transpose(), m_Beq,
                                        m_Aineq.transpose(), m_Bineq,
                                        m_x, m_activeSet, m_activeSetSize);
            
            m_slack = m_x.tail(6);
            return m_x.head(m_n);
        }
    }
}