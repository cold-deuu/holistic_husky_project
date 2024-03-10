#pragma once

#include <holistic_action_manager/server/action_server_base.hpp>
#include <holistic_action_manager/JointPostureAction.h>

#include <fstream>

using namespace pinocchio;
using namespace Eigen;

class JointPostureActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<holistic_action_manager::JointPostureAction> as_;

  holistic_action_manager::JointPostureFeedback feedback_;
  holistic_action_manager::JointPostureResult result_;
  holistic_action_manager::JointPostureGoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;

public:
  JointPostureActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::HuskyFrankaWrapper>  &mu);

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;

protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time);
  
};