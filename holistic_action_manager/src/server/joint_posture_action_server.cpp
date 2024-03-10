#include <holistic_action_manager/server/joint_posture_action_server.hpp>

JointPostureActionServer::JointPostureActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::HuskyFrankaWrapper> &mu)
: ActionServerBase(name,nh,mu), as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&JointPostureActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&JointPostureActionServer::preemptCallback, this));
  as_.start();
}

void JointPostureActionServer::goalCallback()
{
    feedback_header_stamp_ = 0;
    goal_ = as_.acceptNewGoal();
    Eigen::VectorXd q_ref(7);
    for (int i=0; i<7; i++){
      q_ref(i) = goal_->target_joints.position[i];
    }
    ros::Duration dur = ros::Duration(goal_->duration);
    mu_->get_armjoint_task(q_ref);
    start_time_ = ros::Time::now();
    std::cout<<"2"<<std::endl;
    mu_->init_joint_compute(start_time_,dur);
    control_running_ = true;  
}

void JointPostureActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool JointPostureActionServer::compute(ros::Time ctime)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  mu_->compute_arm_control(ctime);

  if (ctime.toSec() - start_time_.toSec() > goal_->duration){
    setSucceeded();
    return true;
  }

  if (ctime.toSec() - start_time_.toSec()  > goal_->duration + 1.0){
    setAborted();
    return false;
  }

  return false;
}


void JointPostureActionServer::signalAbort(bool is_aborted)
{
  setAborted();  
}

void JointPostureActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  control_running_ = false;
}
void JointPostureActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}