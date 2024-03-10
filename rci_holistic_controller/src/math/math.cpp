#include <cmath>
#include <iostream>
#include <Eigen/QR>
#include <Eigen/Dense>
#include "rci_holistic_controller/math/math.hpp"
#include <ros/ros.h>

using namespace holistic_controller::robot;
using namespace Eigen;
using namespace pinocchio;
using namespace std;

namespace holistic_controller{
    namespace math{
        Eigen::MatrixXd pseudoinv(Eigen::MatrixXd Jacob){
            Eigen::MatrixXd eye(6,6);
            eye.setIdentity();
            return Jacob.transpose() *(Jacob*Jacob.transpose()+0.001*eye).inverse();
            // return Jacob.transpose() * (Jacob * Jacob.transpose()).inverse(); //no damp
        }

        double manipulability(Eigen::MatrixXd Jacob)
        {
            return sqrt((Jacob*Jacob.transpose()).determinant());
        }

        Eigen::VectorXd jacobm(double m1, double m2, Eigen::VectorXd q1, Eigen::VectorXd q2)
        {
            Eigen::VectorXd Jm(7);
            for(int i=0; i<7; i++){
                if((q2(i)-q1(i)==0))
                {
                    Jm(i) = 0;
                }
                else{
                    Jm(i) = (m2-m1)/(q2(i) - q1(i));
                }
            }

            return Jm;
        }
        
        Eigen::VectorXd getJacobM(RobotWrapper &robot,Eigen::VectorXd &q,Eigen::VectorXd &v,string ee_id, int &arm_num){
            pinocchio::Data data(robot.model());
            pinocchio::Data data_m1(robot.model()), data_m2(robot.model()), data_m3(robot.model()), data_m4(robot.model()), data_m5(robot.model()), data_m6(robot.model()), data_m7(robot.model());
            Eigen::VectorXd qm(q.size()),q_a(q.size()),q_a1(q.size()),q_a2(q.size()),q_a3(q.size()),q_a4(q.size()),q_a5(q.size()),q_a6(q.size()),q_a7(q.size()),jacob_mpb(q.size()),mpb(q.size());
            Eigen::MatrixXd J(6,q.size()),J_for_m1(6,q.size()),J_for_m2(6,q.size()),J_for_m3(6,q.size()),J_for_m4(6,q.size()),J_for_m5(6,q.size()),J_for_m6(6,q.size()),J_for_m7(6,q.size());
            Eigen::MatrixXd J1(6,7),J_mpb1(6,7),J_mpb2(6,7),J_mpb3(6,7),J_mpb4(6,7),J_mpb5(6,7),J_mpb6(6,7),J_mpb7(6,7);
            
            double m;
            double eps = 1e-5;
            int mobile =  q.size() - arm_num;
            robot.computeAllTerms(data,q,v);
            
            robot.jacobianWorld(data,robot.model().getJointId("panda_joint7"),J);
            J1 = J.topRightCorner(6,7);
            m = manipulability(J1);

            q_a = q;
            q_a1 = q;
            q_a2 = q;
            q_a3 = q;
            q_a4 = q;
            q_a5 = q;
            q_a6 = q;
            q_a7 = q;

            q_a1(mobile+0) += eps;
            q_a2(mobile+1) += eps;
            q_a3(mobile+2) += eps;
            q_a4(mobile+3) += eps;
            q_a5(mobile+4) += eps;
            q_a6(mobile+5) += eps;
            q_a7(mobile+6) += eps;

            robot.computeAllTerms(data_m1,q_a1,v);
            robot.jacobianWorld(data_m1,robot.model().getJointId("panda_joint7"),J_for_m1);
            robot.computeAllTerms(data_m2,q_a2,v);
            robot.jacobianWorld(data_m2,robot.model().getJointId("panda_joint7"),J_for_m2);
            robot.computeAllTerms(data_m3,q_a3,v);
            robot.jacobianWorld(data_m3,robot.model().getJointId("panda_joint7"),J_for_m3);
            robot.computeAllTerms(data_m4,q_a4,v);
            robot.jacobianWorld(data_m4,robot.model().getJointId("panda_joint7"),J_for_m4);
            robot.computeAllTerms(data_m5,q_a5,v);
            robot.jacobianWorld(data_m5,robot.model().getJointId("panda_joint7"),J_for_m5);
            robot.computeAllTerms(data_m6,q_a6,v);
            robot.jacobianWorld(data_m6,robot.model().getJointId("panda_joint7"),J_for_m6);
            robot.computeAllTerms(data_m7,q_a7,v);
            robot.jacobianWorld(data_m7,robot.model().getJointId("panda_joint7"),J_for_m7);
            
            J_mpb1 = J_for_m1.topRightCorner(6,7);
            J_mpb2 = J_for_m2.topRightCorner(6,7);
            J_mpb3 = J_for_m3.topRightCorner(6,7);
            J_mpb4 = J_for_m4.topRightCorner(6,7);
            J_mpb5 = J_for_m5.topRightCorner(6,7);
            J_mpb6 = J_for_m6.topRightCorner(6,7);
            J_mpb7 = J_for_m7.topRightCorner(6,7);


            jacob_mpb(0) = manipulability(J_mpb1);
            jacob_mpb(1) = manipulability(J_mpb2);
            jacob_mpb(2) = manipulability(J_mpb3);
            jacob_mpb(3) = manipulability(J_mpb4);
            jacob_mpb(4) = manipulability(J_mpb5);
            jacob_mpb(5) = manipulability(J_mpb6);
            jacob_mpb(6) = manipulability(J_mpb7);

            for(int i=0; i<7;i++){
                mpb(i) = (jacob_mpb(i)-m)/eps;
            }

            return mpb;
            
        }

        Eigen::VectorXd get_error_6d(pinocchio::SE3 &oMi, pinocchio::SE3 &goal_se3){
            // pinocchio::SE3 dMi = oMi.inverse() * goal_se3;
            Eigen::MatrixXd rot_diff = oMi.rotation().inverse() * goal_se3.rotation();
            Eigen::Vector3d delphi = GetPhi(oMi.rotation(), goal_se3.rotation());

            Eigen::VectorXd err(6);
            err.head(3) = goal_se3.translation() - oMi.translation();
            // err.tail(3) = pinocchio::log3(rot_diff);
            err.tail(3) = -delphi;
            return err;
        }

        Matrix3d AngleAngle_to_Rot(Vector3d axis, double angle) {
            Matrix3d Rot;
            double kx, ky, kz, theta, vt;
            kx = axis(0);
            ky = axis(1);
            kz = axis(2);
            theta = angle;
            vt = 1.0 - cos(theta);


            Rot(0, 0) = kx * kx*vt + cos(theta);
            Rot(0, 1) = kx * ky*vt - kz * sin(theta);
            Rot(0, 2) = kx * kz*vt + ky * sin(theta);
            Rot(1, 0) = kx * ky*vt + kz * sin(theta);
            Rot(1, 1) = ky * ky*vt + cos(theta);
            Rot(1, 2) = ky * kz*vt - kx * sin(theta);
            Rot(2, 0) = kx * kz*vt - ky * sin(theta);
            Rot(2, 1) = ky * kz*vt + kx * sin(theta);
            Rot(2, 2) = kz * kz*vt + cos(theta);

            return Rot;
        }

        Vector3d GetPhi(Matrix3d Rot, Matrix3d Rotd)
        {
        Vector3d phi;
        Vector3d s[3], v[3], w[3];
        for (int i = 0; i < 3; i++) {
        v[i] = Rot.block(0, i, 3, 1);
        w[i] = Rotd.block(0, i, 3, 1);
        s[i] = v[i].cross(w[i]);
        }
        phi = s[0] + s[1] + s[2];
        phi = -0.5* phi;
        return phi;
        }
    }
}

