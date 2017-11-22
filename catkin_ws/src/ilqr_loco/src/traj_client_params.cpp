#include "traj_client.h"

void TrajClient::LoadParams()
{
  try
  {
    ROS_INFO("Loading parameters.");

    // Get parameters from ROS Param server
    TRYGETPARAM("timestep", timestep_)

    TRYGETPARAM("kp_ramp", kp_)
    TRYGETPARAM("ki_ramp", ki_)
    TRYGETPARAM("kd_ramp", kd_)
	  TRYGETPARAM("kp_ramp_y", kp_y_)
    TRYGETPARAM("accel_ramp", accel_)
    TRYGETPARAM("target_vel_ramp", target_vel_)
    TRYGETPARAM("pre_ramp_vel", pre_ramp_vel_)
    TRYGETPARAM("pre_ramp_time", pre_ramp_time_)
    TRYGETPARAM("timeout_ramp", timeout_)
    TRYGETPARAM("ramp_steer_multiplier", ramp_steer_multiplier_)

    TRYGETPARAM("use_pid", use_pid_);

    TRYGETPARAM("X_des", x_des_)
    TRYGETPARAM("timeout_ilqr_mpc", mpc_timeout_)
    TRYGETPARAM("stop_goal_threshold", goal_threshold_)
    TRYGETPARAM("use_extrapolate", use_extrapolate_)
    TRYGETPARAM("extrapolate_dt", extrapolate_dt_)

    TRYGETPARAM("ilqr_tolFun", ilqr_tolFun_)
    TRYGETPARAM("ilqr_tolConstraint", ilqr_tolConstraint_)
    TRYGETPARAM("ilqr_tolGrad", ilqr_tolGrad_)
    TRYGETPARAM("ilqr_max_iter", ilqr_max_iter_)
    TRYGETPARAM("ilqr_regType", ilqr_regType_)
    TRYGETPARAM("ilqr_debug_level", ilqr_debug_level_)

    TRYGETPARAM("init_control_seq", init_control_seq_)
	  u_seq_saved_ = init_control_seq_;
	T_horizon_ = init_control_seq_.size()/2;

	TRYGETPARAM("naive_obstacle_dist_thres", obs_dist_thres_)
  double lidar_to_bumper_dist = 0.1;
  obs_dist_thres_ += lidar_to_bumper_dist;

	TRYGETPARAM("naive_obstacle_percent_thres", obs_percent_thres_)
	TRYGETPARAM("scan_clip_angle", scan_front_angle_)

	TRYGETPARAM("use_fixed_rate_replan", use_fixed_rate_replan_)
  TRYGETPARAM("replan_rate", replan_rate_)

    LoadOpt();
  }
  catch(...)
  {
    ROS_ERROR("Please put all params into yaml file, and load it.");
  }
}

void TrajClient::LoadCarParams()
{
  TRYGETPARAM("Opt_car_param/g", g_);
  TRYGETPARAM("Opt_car_param/L", L_);
  TRYGETPARAM("Opt_car_param/m", m_);
  TRYGETPARAM("Opt_car_param/b", b_);
  TRYGETPARAM("Opt_car_param/c_x", c_x_);
  TRYGETPARAM("Opt_car_param/c_a", c_a_);
  TRYGETPARAM("Opt_car_param/Iz", Iz_);
  TRYGETPARAM("Opt_car_param/mu", mu_);
  TRYGETPARAM("Opt_car_param/mu_s", mu_s_);
  TRYGETPARAM("Opt_car_param/limSteer", limSteer_);
  TRYGETPARAM("Opt_car_param/limThr", limThr_);

  a_ = L_ - b_;
  G_f_ = m_*g_*b_/L_;
  G_r_ = m_*g_*a_/L_;
}

void TrajClient::LoadCostParams()
{
  TRYGETPARAM("Opt_cost/cu", cu_)
  TRYGETPARAM("Opt_cost/cdu", cdu_)
  TRYGETPARAM("Opt_cost/cf", cf_)
  TRYGETPARAM("Opt_cost/pf", pf_)
  TRYGETPARAM("Opt_cost/cx", cx_)
  TRYGETPARAM("Opt_cost/cdx", cdx_)
  TRYGETPARAM("Opt_cost/px", px_)
  TRYGETPARAM("Opt_cost/k_pos", k_pos_)
  TRYGETPARAM("Opt_cost/d_thres", d_thres_)
  TRYGETPARAM("Opt_cost/Obs_offsetX", Obs_offsetX_)
  TRYGETPARAM("Opt_cost/Obs_offsetY", Obs_offsetY_)
  k_pos2_ = 7.0;
  k_pos3_ = 7.0;
  k_pos4_ = 7.0;
  d_thres2_ = 0.5;
  d_thres3_ = 0.5;
  d_thres4_ = 0.5;
}

// changed to pass by reference to apply and keep edit
void TrajClient::SetOptParams(tOptSet *o)
{
    o->tolFun= ilqr_tolFun_;
    o->tolConstraint= ilqr_tolConstraint_;
    o->tolGrad= ilqr_tolGrad_;
    o->max_iter= ilqr_max_iter_;
    o->regType= ilqr_regType_;
    o->debug_level= ilqr_debug_level_;

    o->n_alpha= 8;
    o->lambdaInit= 1;
    o->dlambdaInit= 1;
    o->lambdaFactor= 1.6;
    o->lambdaMax= 0.0000000001;
    o->lambdaMin= 0.000001;
    o->zMin= 0.0;
    o->w_pen_init_l= 1.0;
    o->w_pen_init_f= 1.0;
    o->w_pen_max_l= INF;
    o->w_pen_max_f= INF;
    o->w_pen_fact1= 4.0; // 4...10 Bertsekas p. 123
    o->w_pen_fact2= 1.0;
}

void TrajClient::LoadOpt()
{
  LoadCarParams();
  LoadCostParams();

  Opt = INIT_OPTSET;

  SetOptParams(&Opt);

  Opt.p= (double **) malloc(n_params*sizeof(double *));

  Opt.p[0] = assignPtrVal(&G_f_,1);
  Opt.p[1] = assignPtrVal(&G_r_,1);
  Opt.p[2] = assignPtrVal(&Iz_,1);
  // [3] Obs
  Opt.p[4] = assignPtrVal(&Obs_offsetX_[0],3);
  Opt.p[5] = assignPtrVal(&Obs_offsetY_[0],3);
  Opt.p[6] = assignPtrVal(&a_,1);
  Opt.p[7] = assignPtrVal(&b_,1);
  Opt.p[8] = assignPtrVal(&c_a_,1);
  Opt.p[9] = assignPtrVal(&c_x_,1);
  Opt.p[10] = assignPtrVal(&cdu_[0],2);
  Opt.p[11] = assignPtrVal(&cf_[0],3);
  Opt.p[12] = assignPtrVal(&cu_[0],2);
  Opt.p[13] = assignPtrVal(&cx_[0],3);
  Opt.p[14] = assignPtrVal(&d_thres_,1);
  Opt.p[15] = assignPtrVal(&d_thres2_,1);
  Opt.p[16] = assignPtrVal(&d_thres3_,1);
  Opt.p[17] = assignPtrVal(&d_thres4_,1);

  Opt.p[18] = assignPtrVal(&timestep_,1);
  Opt.p[19] = assignPtrVal(&k_pos_,1);
  Opt.p[20] = assignPtrVal(&k_pos2_,1);
  Opt.p[21] = assignPtrVal(&k_pos3_,1);
  Opt.p[22] = assignPtrVal(&k_pos4_,1);

  Opt.p[23] = assignPtrVal(&limSteer_[0],2);
  Opt.p[24] = assignPtrVal(&limThr_[0],2);
  Opt.p[25] = assignPtrVal(&m_,1);
  Opt.p[26] = assignPtrVal(&mu_,1);
  Opt.p[27] = assignPtrVal(&mu_s_,1);
  Opt.p[28] = assignPtrVal(&pf_[0],3);
  Opt.p[29] = assignPtrVal(&px_[0],3);
  // [30] xDes

  char *err_msg;
}
