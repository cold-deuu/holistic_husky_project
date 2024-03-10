#pragma once

#include <holistic_action_manager/server/action_server_base.hpp>
#include <holistic_action_manager/HolisticAction.h>
#include "tf/transform_datatypes.h"

#include <fstream>

using namespace pinocchio;
using namespace Eigen;

class HolisticActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<holistic_action_manager::HolisticAction> as_;

  holistic_action_manager::HolisticFeedback feedback_;
  holistic_action_manager::HolisticResult result_;
  holistic_action_manager::HolisticGoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;

public:
  HolisticActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::HuskyFrankaWrapper>  &mu);
  ros::Duration dur_;

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;

protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time);
};