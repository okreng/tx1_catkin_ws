/*
Executes most recent trajectory given by iLQR.
Sends feedback to action client (planner) in terms of "almost done", "done".
*/

#include "traj_server.h"

void TrajServer::LoadParams()
{
	ROS_INFO("Loading traj server params.");
    TRYGETPARAM("old_msg_discard_thres", old_msg_thres)
    TRYGETPARAM("kp_heading", kp_)
    TRYGETPARAM("ki_heading", ki_)
    TRYGETPARAM("kd_heading", kd_)
    std::vector<double> goal_state;
    TRYGETPARAM("X_des", goal_state)
    goal_x_ = goal_state[0];
    ROS_INFO("TrajServer: goal x= %f", goal_x_);
}

void TrajServer::PublishPath(const ilqr_loco::TrajExecGoalConstPtr &goal)
{
  nav_msgs::Path path_msg;
  path_msg.header.stamp = goal->traj.header.stamp;
  path_msg.header.frame_id = "map";
  std::vector<geometry_msgs::PoseStamped> poses(goal->traj.states.size());

  for (int i=0; i < goal->traj.commands.size(); i++)
  {
    poses.at(i).pose.position.x = goal->traj.states[i].pose.pose.position.x;
    poses.at(i).pose.position.y = goal->traj.states[i].pose.pose.position.y;
    poses.at(i).pose.orientation = goal->traj.states[i].pose.pose.orientation;
  }
  path_msg.poses = poses;
  path_pub.publish(path_msg);
  ros::spinOnce();
}

void TrajServer::stateCb(const nav_msgs::Odometry &msg)
{
  cur_x_ = msg.pose.pose.position.x;
  cur_yaw_  = tf::getYaw(msg.pose.pose.orientation);
}

geometry_msgs::Twist TrajServer::pid_correct_yaw(geometry_msgs::Twist orig_twist, nav_msgs::Odometry state)
{
  double yaw_des = tf::getYaw(state.pose.pose.orientation);
  double dt = (state.header.stamp).toSec() - t_;
  t_ = (state.header.stamp).toSec();
  double orig_steer = orig_twist.angular.z;

  // Correct steering to compensate for yaw error
  double yaw_error = yaw_des - cur_yaw_;
  cur_integral_ += yaw_error*dt;

  double p = kp_*yaw_error;
  double i = clamp(ki_*cur_integral_, -0.25, 0.25);
  double d = clamp(kd_*(yaw_error-prev_error_)/dt, -0.1, 0.1);
  double correction =  p + i + d;

  double steer = orig_steer + correction;
  prev_error_ = yaw_error;

  geometry_msgs::Twist new_twist;
  new_twist.linear.x = orig_twist.linear.x;
  new_twist.angular.z = steer;

  // ROS_INFO("TrajServer: P: %.2f | I: %.2f  | D: %.2f | out: %.2f", p, i, d, correction);

  prev_error_ = yaw_error;

  return new_twist;
}

// provides action to execute plans
void TrajServer::execute_trajectory(const ilqr_loco::TrajExecGoalConstPtr &goal){
  // TODO? check that states and commands are right length
  // ROS_INFO("%s: Received trajectory.", traj_action.c_str());

  bool success = true;

  double timestep = goal->traj.timestep;
  double traj_start_time = (goal->traj.header.stamp).toSec();

  ros::Rate loop_rate(1.0/timestep);
  // ROS_INFO("Executing trajectory.");

  PublishPath(goal);
  // ROS_INFO("Finished publishing path.");

  if ( (cur_x_ - goal_x_) > 0.1 )
  {
    // Make sure robot doesn't run away past the goal point
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.angular.z = 0;
    cmd_pub.publish(twist);
    ROS_INFO("Passed goal point.");
    result_.done = 0;
    as.setSucceeded(result_);
    return;
  }

  for (int i=0; i < goal->traj.commands.size(); i++)
  {
    // check that preempt has not been requested by the client
    if (as.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted at %dth command.", traj_action.c_str(), i);
      as.setPreempted();
      success = false;
      break;
    }
    // check that commands in plan are not too old
    else if ( (ros::Time::now().toSec() - (traj_start_time + (i*timestep)) ) > old_msg_thres
                || (goal->traj.commands.size()>1 && i<3 && goal->traj.execution_mode == 1) ) // dirty way to skip first 3 commands of ilqr traj
    {
     ROS_INFO("%s: Ignoring old command. steering: %f", traj_action.c_str(), goal->traj.commands[i].angular.z);
     continue;
    }
    // else, execute command!
    else
    {
      if (goal->traj.execution_mode == 1 && i < goal->traj.commands.size()-1)
      {
        if (goal->traj.commands[i].linear.x == 0)
        {
          ROS_INFO("Zero twist.");
          cmd_pub.publish(goal->traj.commands[i]);
        }
        else
        {
          geometry_msgs::Twist pid_twist = pid_correct_yaw(goal->traj.commands[i], goal->traj.states[i]);
          ROS_INFO("iLQR Twist: %f, %f", pid_twist.linear.x, pid_twist.angular.z);
          cmd_pub.publish(pid_twist);
        }
      }
      else
      {
		// ROS_INFO("Command: %f, %f", goal->traj.commands[i].linear.x, goal->traj.commands[i].angular.z);
        if (goal->traj.commands.size() == 1)
        {
          ROS_INFO("Ramp twist.");
        }
        else
        {
          ROS_INFO("Playback twist.");
        }

        cmd_pub.publish(goal->traj.commands[i]);
      }

  	  feedback_.step = i;
  	  as.publishFeedback(feedback_);
      ros::spinOnce();

      int steps_left = goal->traj.commands.size() - i;
      if (steps_left>1)
        loop_rate.sleep();
    }
  }

  if (success)
  {
    // ROS_INFO("%s: Finished publishing trajectory`", traj_action.c_str());
    result_.done = success;
    as.setSucceeded(result_);
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "traj_server");

  TrajServer executer;
  ros::spin();

  return 0;
}
