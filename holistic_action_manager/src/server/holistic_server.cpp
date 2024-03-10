#include <holistic_action_manager/server/holistic_server.hpp>
#include <pinocchio/fwd.hpp>

using namespace pinocchio;

HolisticActionServer::HolisticActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::HuskyFrankaWrapper> &mu)
: ActionServerBase(name,nh,mu), as_(nh,name,false)
{
    as_.registerGoalCallback(boost::bind(&HolisticActionServer::goalCallback, this));
    as_.registerPreemptCallback(boost::bind(&HolisticActionServer::preemptCallback, this));
    as_.start();
}

void HolisticActionServer::goalCallback()
{
    feedback_header_stamp_ = 0;
    goal_ = as_.acceptNewGoal();
    
    Eigen::Vector3d pos(goal_->target_pose.position.x, goal_->target_pose.position.y, goal_->target_pose.position.z);
    Eigen::Quaterniond quat(goal_->target_pose.orientation.w, goal_->target_pose.orientation.x, goal_->target_pose.orientation.y, goal_->target_pose.orientation.z);
    SE3 oMi_ref(quat.toRotationMatrix(), pos);
    mu_ -> get_se3_task(oMi_ref);
    dur_ = ros::Duration(goal_->duration); 
    start_time_ = ros::Time::now();
    mu_->init_holistic_compute(start_time_,dur_);
    control_running_ = true;  
}

void HolisticActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool HolisticActionServer::compute(ros::Time ctime)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  mu_->compute_holistic(ctime);
  
  if (ctime.toSec() - start_time_.toSec() > goal_->duration + 1.0 && mu_->get_pub().pose_err <0.005 ){
    setSucceeded();
    return true;
  }

  if (ctime.toSec() - start_time_.toSec()  > goal_->duration+ 2.0){
    setAborted();
    return false;
  }

  return false;
}


void HolisticActionServer::signalAbort(bool is_aborted)
{
  setAborted();  
}

void HolisticActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  control_running_ = false;
}
void HolisticActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}