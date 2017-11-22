#include "traj_client.h"

ilqr_loco::TrajExecGoal TrajClient::rampGenerateTrajectory(nav_msgs::Odometry prev_state,
                                                           nav_msgs::Odometry cur_state) {

  double dt = (cur_state.header.stamp).toSec() - (prev_state.header.stamp).toSec();
  double yaw = tf::getYaw(cur_state.pose.pose.orientation);
  // ROS_INFO("Yaw = %f", yaw);

  // PID control for vehicle heading
  double y_error = (ramp_start_y_ - cur_state_.pose.pose.position.y);
  double yaw_des = kp_y_*y_error;
  // ROS_INFO("yaw_des = %.2f, y_error = %.2f", yaw_des, y_error);
  // ROS_INFO("output_y: %f, kp_y_: %f", kp_y_*y_error, kp_y_);

  double error = yaw_des - yaw;
  cur_integral_ += error*dt;
  double p = kp_*error;
  double i = clamp(ki_*cur_integral_, -0.25, 0.25);
  double d = clamp(kd_*(error-prev_error_), -0.1, 0.1);
  double output =  clamp(p + i + d, -0.77, 0.77);
  // ROS_INFO("err = %.2f, P = %.2f,  |  I = %.2f,  |  D = %.2f | output = %.2f", error, p, i, d, output);
  // ROS_INFO("Output = %f", output);

  prev_error_ = error;

  // Generate goal
  double v;
  ros::Duration time_since_start = cur_state.header.stamp - start_time_;
  if (time_since_start.toSec() < pre_ramp_time_)
  {
    output *= ramp_steer_multiplier_;
    ROS_INFO("Rampup steer: %.2f", output);
    v = pre_ramp_vel_;
  }
  else
  {
    // v = cur_state.twist.twist.linear.x + accel_*dt+ 0.25;
    v = accel_ * (time_since_start.toSec() - pre_ramp_time_) + pre_ramp_vel_;
    v = clamp(v, 0, target_vel_);
    // ROS_INFO("v = %f", v);
  }

  ilqr_loco::TrajExecGoal goal;

  geometry_msgs::Twist control_msg;
  FillTwistMsg(control_msg, v, output);
  goal.traj.commands.push_back(control_msg);

  nav_msgs::Odometry state_msg;
  double expected_x = start_state_.pose.pose.position.x
                      + pre_ramp_vel_*pre_ramp_time_
                      + 0.5*accel_*start_time_.toSec()*start_time_.toSec();
  double expected_y = start_state_.pose.pose.position.y;

  FillOdomMsg(state_msg, expected_x, expected_y, 0, v, 0, 0); //0s: yaw, vy, w
  goal.traj.states.push_back(state_msg);

  ++T_;
  ramp_goal_flag_ = (v >= target_vel_) ? true : false;  // Ramp completion flag
  FillGoalMsgHeader(goal);

  return goal;
}


void TrajClient::rampPlan() {

  if(ros::Time::now() - start_time_ < ros::Duration(timeout_)) {
    ilqr_loco::TrajExecGoal goal = rampGenerateTrajectory(prev_state_, cur_state_);
    SendTrajectory(goal);
  }
  else {
    // Stop car after ramp timeout
    ROS_INFO("Timeout exceeded, stopping car");
    SendZeroCommand();
    mode_ = 0;
  }
}
