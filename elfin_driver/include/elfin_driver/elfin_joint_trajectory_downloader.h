/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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

#ifndef ELFIN_JOINT_TRAJECTORY_DOWNLOADER_H
#define ELFIN_JOINT_TRAJECTORY_DOWNLOADER_H

#include "industrial_robot_client/joint_trajectory_interface.h"
#include "trajectory_msgs/JointTrajectory.h"

namespace elfin_driver
{
namespace elfin_joint_trajectory_downloader
{

using industrial_robot_client::joint_trajectory_interface::JointTrajectoryInterface;
using industrial::joint_traj_pt_message::JointTrajPtMessage;

/**
 * \brief Message handler that downloads joint trajectories to
 * a robot controller that supports the trajectory downloading interface
 */
class ElfinJointTrajectoryDownloader : public JointTrajectoryInterface
{

  ros::Publisher pub_joint_trajectory_;
  
public:

  /**
   * \brief Initialize required services and topics.
   *
    *
   * \return true on success, false otherwise
   */
   bool init();

  /**
   * \brief Send trajectory to robot, using this node's robot-connection.
   *   Specific method must be implemented in a derived class (e.g. streaming, download, etc.)
   *
   * \param messages List of SimpleMessage JointTrajPtMessages to send to robot.
   *
   * \return true on success, false otherwise
   */
   bool send_to_robot(const std::vector<JointTrajPtMessage>& messages){};
   
  /**
   * \brief Send trajectory to robot, using this node's robot-connection.
   *   Specific method must be implemented in a derived class (e.g. streaming, download, etc.)
   *
   * \param msg Pointer to trajectory points,JointTrajectoryConstPtr to send to robot.
   *
   * \return true on success, false otherwise
   */

  bool send_to_robot(const trajectory_msgs::JointTrajectoryConstPtr &msg);

  /**
   * \brief Callback function registered to ROS topic-subscribe.
   *   Transform message into SimpleMessage objects and send commands to robot.
   *
   * \param msg JointTrajectory message from ROS trajectory-planner
   */
  void jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg);
 
  /**
   * \brief Callback function registered to ROS CmdJointTrajectory service
   *   Duplicates message-topic functionality, but in service form.
   *
   * \param req CmdJointTrajectory request from service call
   * \param res CmdJointTrajectory response to service call
   * \return true always.  Look at res.code.val to see if call actually succeeded
   */
  bool jointTrajectoryCB(industrial_msgs::CmdJointTrajectory::Request &req,
                         industrial_msgs::CmdJointTrajectory::Response &res);

  /**
   * \brief Callback function registered to ROS stopMotion service
   *   Sends stop-motion command to robot.
   *
   * \param req StopMotion request from service call
   * \param res StopMotion response to service call
   * \return true always.  Look at res.code.val to see if call actually succeeded.
   */
  virtual bool stopMotionCB(industrial_msgs::StopMotion::Request &req,
                                    industrial_msgs::StopMotion::Response &res);  
};

} //elfin_joint_trajectory_downloader
} //elfin_driver

#endif /* ELFIN_JOINT_TRAJECTORY_DOWNLOADER_H */