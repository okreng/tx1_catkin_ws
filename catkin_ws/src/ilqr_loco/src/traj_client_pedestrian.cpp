#include "traj_client.h"
#include <array>
#define STOPATDISTANCE stop_within_=5;

// This file is modeled off of the traj_client_detector file
// This file contains pedestrianCb, which is the callback function
// for the pedestrian topic
// The goal is to check if the pedestrian is (will be) within range of the vehicle
// If so, this function will call the reactToObstacle function

// TODO: Change this function to InitPedestrian to define any variables needed
//void TrajClient::InitDetector()
//{
//  double new_angle_min = -scan_front_angle_/2;
//  double new_angle_max = scan_front_angle_/2;
//  double old_angle = 2.35619449615;
//  double increment = 0.00436332309619;

// scan_min_index_ = floor((old_angle-new_angle_max) / increment);
//  scan_max_index_ = 1080 - scan_min_index_;
//}

// Modeled off of TransformLaserToMap
// Transforms the pedestrian trajectory from infrastructure to map
// This function is complete, as it will transform points individually
bool TrajClient::TransformInfrastructureToMap(geometry_msgs::PointStamped &ped_inf_frame,
                                              geometry_msgs::PointStamped &ped_map_frame)
{
  try
  {
    tf::StampedTransform transform;
    tf_listener_.waitForTransform("map", "infrastructure", ros::Time::now(), ros::Duration(0.01));
    tf_listener_.lookupTransform("map", "infrastructure", ros::Time(0), transform);
    tf_listener_.transformPoint("map", ped_inf_frame, ped_map_frame);
    return true;
  }
  catch (...)
  {
    ROS_INFO("Pedestrian detector: Infrastructure to laser transform not available.");
    return false;
  }
}

// // TODO: change message type to geometry_msgs::PointStampedListConstPtr
// void TrajClient::pedestrianCb(const geometry_msgs::PointStampedConstPtr &msg)
// {
//   // ROS_INFO("start pedestrian detector");

//   if(found_obstacle_) return;

//   // Look for obstacles in slice of scan
//   int n_scans_close_enough = 0;
//   double ped_dist;

//   // TODO: change this to become an iterator through the list of points
//   //for(int i=scan_min_index_; i<scan_max_index_; i++)
//   //{
//     if (msg->point.x<obs_dist_thres_)
//     {
//       //n_scans_close_enough++;
//       ped_dist = msg->point.x;
//       ROS_INFO("Pedestrian detector: Found pedestrian.");
//       geometry_msgs::PointStamped ped_pos_infraframe;
//       geometry_msgs::PointStamped ped_pos_mapframe;
//       ped_pos_infraframe.header.frame_id = "infrastructure";
//       ped_pos_mapframe.header.frame_id = "map";

//       double yaw = tf::getYaw(cur_state_.pose.pose.orientation);
//       ped_pos_infraframe.point.x = ped_dist * cos(yaw);
//       ped_pos_infraframe.point.y = 0;

//       // TODO: Determine where the variable obs_pos_ is initialized and cp for ped
//       if (TransformInfrastructureToMap(ped_pos_infraframe, ped_pos_mapframe))
//       {
//         ped_pos_.x = ped_pos_mapframe.point.x;
//         ped_pos_.y = 0; // hack to deal with heading uncertainty
//         ped_pos_mapframe.point.y = 0; // hack to deal with heading uncertainty
//         found_obstacle_ = true; // keep this variable unchanged as it refers to a
//                                 // global variable within ReactToObstacle
//         ped_pos_pub_.publish(ped_pos_mapframe);

//         ROS_INFO("Transformed pedestrian pos.");

//         if (!reacted_to_obstacle_)
//           ReactToObstacle();

//         ros::spinOnce();


//       }

//     }
//   //}

//   // For our purposes we do not need to check a percentage, if a single point
//   // is close enough we will activate the ReactToObstacle function
//   //float percent_scans_close = float(n_scans_close_enough)/float(scan_max_index_ - scan_min_index_);

//   // Fill data, transform, send cluster_center_pos here
//   //if(percent_scans_close > obs_percent_thres_)
//   //{




//   //}
// }

//void TrajClient::pedestrianCb(const infrastructure_to_vehicle::PointArray &pointArray)
void TrajClient::pedestrianCb(const geometry_msgs::PoseArray &poseArray)
{
   ROS_INFO("start pedestrian detector");

  if(found_obstacle_) return;

  // Look for obstacles in slice of scan
  int n_scans_close_enough = 0;
  double ped_dist;

  // TODO: change this to become an iterator through the list of points
  //for(int i=scan_min_index_; i<scan_max_index_; i++)
  //{

  for (int ii=0; ii < sizeof(poseArray.poses); ii++)
  {
    geometry_msgs::Point pedTime = poseArray.poses[ii].position;
  //  if (pedTime.x<obs_dist_thres_)
  //  {
      //n_scans_close_enough++;
      ped_dist = pedTime.x;
      ROS_INFO("Pedestrian detector: Found pedestrian.");
      geometry_msgs::PointStamped ped_pos_infraframe;
      geometry_msgs::PointStamped ped_pos_mapframe;
      ped_pos_infraframe.header.frame_id = "infrastructure";
      ped_pos_mapframe.header.frame_id = "map";

      // double yaw = tf::getYaw(cur_state_.pose.pose.orientation);
      // ped_pos_infraframe.point.x = ped_dist * cos(yaw);
      // ped_pos_infraframe.point.y = 0;

      ped_pos_infraframe.point.x = pedTime.x;
      ped_pos_infraframe.point.y = pedTime.y;

      // TODO: Determine where the variable obs_pos_ is initialized and cp for ped
      if (TransformInfrastructureToMap(ped_pos_infraframe, ped_pos_mapframe))
      {
        ped_pos_.x = ped_pos_mapframe.point.x;
        ped_pos_.y = ped_pos_mapframe.point.y; // hack to deal with heading uncertainty
        // ped_pos_mapframe.point.y = ped_pos_mapframe.point.y; // hack to deal with heading uncertainty
        found_obstacle_ = true; // keep this variable unchanged as it refers to a
                                // global variable within ReactToObstacle
        ped_pos_pub_.publish(ped_pos_mapframe);

        ROS_INFO("Transformed pedestrian pos.");

	if (abs(ped_pos_.x-cur_state_.pose.pose.position.x) < stop_within_)
	  {
          ROS_INFO("Reacting to pedestrian trajectory.");
          if (!reacted_to_obstacle_)
            ReactToObstacle();

          ros::spinOnce();
	}

      }

//    }
  }
  //}

  // For our purposes we do not need to check a percentage, if a single point
  // is close enough we will activate the ReactToObstacle function
  //float percent_scans_close = float(n_scans_close_enough)/float(scan_max_index_ - scan_min_index_);

  // Fill data, transform, send cluster_center_pos here
  //if(percent_scans_close > obs_percent_thres_)
  //{




  //}
}


