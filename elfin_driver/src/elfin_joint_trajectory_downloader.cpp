/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "elfin_driver/elfin_joint_trajectory_downloader.h"

namespace elfin_driver
{
namespace elfin_joint_trajectory_downloader
{

using industrial::simple_message::SimpleMessage;
namespace SpecialSeqValues = industrial::joint_traj_pt::SpecialSeqValues;

bool ElfinJointTrajectoryDownloader::send_to_robot(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
  bool rslt=true;

  pub_joint_trajectory_.publish(msg);

  return rslt;
}

bool ElfinJointTrajectoryDownloader::init()
{
//  this->all_joint_names_ = joint_names;
//  this->joint_vel_limits_ = velocity_limits;

  // try to read velocity limits from URDF, if none specified
//  if (joint_vel_limits_.empty() && !industrial_utils::param::getJointVelocityLimits("robot_description", joint_vel_limits_))
//    ROS_WARN("Unable to read velocity limits from 'robot_description' param.  Velocity validation disabled.");

  this->srv_stop_motion_ = this->node_.advertiseService("stop_motion", &ElfinJointTrajectoryDownloader::stopMotionCB, this);
  this->srv_joint_trajectory_ = this->node_.advertiseService("joint_path_command", &ElfinJointTrajectoryDownloader::jointTrajectoryCB, this);
  this->sub_joint_trajectory_ = this->node_.subscribe("joint_path_command", 0, &ElfinJointTrajectoryDownloader::jointTrajectoryCB, this);
  //this->sub_cur_pos_ = this->node_.subscribe("joint_states", 1, &ElfinJointTrajectoryDownloader::jointStateCB, this);
  
  // Topic for sending commands to the robot via the non ROS Industrial driver
  pub_joint_trajectory_ = node_.advertise<trajectory_msgs::JointTrajectory>("elfin_arm_controller/command", 1);

  return true;
}

bool ElfinJointTrajectoryDownloader::jointTrajectoryCB(industrial_msgs::CmdJointTrajectory::Request &req,
                                                 industrial_msgs::CmdJointTrajectory::Response &res)
{
  trajectory_msgs::JointTrajectoryPtr traj_ptr(new trajectory_msgs::JointTrajectory);
  *traj_ptr = req.trajectory;  // copy message data
  this->jointTrajectoryCB(traj_ptr);

  // no success/fail result from jointTrajectoryCB.  Assume success.
  res.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;

  return true;  // always return true.  To distinguish between call-failed and service-unavailable.
}

void ElfinJointTrajectoryDownloader::jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
  ROS_INFO("Receiving joint trajectory message on Elfin");

  // check for STOP command
  if (msg->points.empty())
  {
    ROS_INFO("Empty trajectory received, canceling current trajectory");
    trajectoryStop();
    return;
  }

  // publish command messages to robot
  send_to_robot(msg);
}

bool ElfinJointTrajectoryDownloader::stopMotionCB(industrial_msgs::StopMotion::Request &req,
		                                    industrial_msgs::StopMotion::Response &res)
{
  trajectoryStop();

  // no success/fail result from trajectoryStop.  Assume success.
  res.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;

  return true;  // always return true.  To distinguish between call-failed and service-unavailable.
}

} //elfin_joint_trajectory_downloader
} //elfin_driver