/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, 2020 Canonical Robots
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
 * 	* Neither the name of the Canonical Robots, nor the names
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

#include "elfin_driver/elfin_robot_joint_relay_handler.h"
#include "industrial_robot_client/robot_status_relay_handler.h"
#include "sensor_msgs/JointState.h"
#include "simple_message/log_wrapper.h"

using namespace industrial::shared_types;
using namespace industrial::smpl_msg_connection;
using namespace industrial::simple_message;
using namespace industrial::robot_status;
using namespace industrial::robot_status_message;

namespace elfin_driver
{
namespace elfin_robot_joint_relay_handler
{

bool ElfinRobotJointRelayHandler::init()
{
  sub_joint_states_ = node_.subscribe("/elfin_arm_controller/follow_joint_trajectory/feedback", 1000, &ElfinRobotJointRelayHandler::jointCB, this);
  
  //pub_joint_states_ = node_.advertise<sensor_msgs::JointState>("joint_states", 1);
  pub_joint_trajectory_ = node_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 1);

  return true;
}

bool ElfinRobotJointRelayHandler::internalCB(SimpleMessage & in)
{
	ROS_ERROR("Not implemented");
	return false;
}



void ElfinRobotJointRelayHandler::jointCB(const control_msgs::FollowJointTrajectoryFeedback & in)
{

  sensor_msgs::JointState state;
  control_msgs::FollowJointTrajectoryFeedback trajectory_feedback;
  
//  bool rtn = true;

  state.header.stamp = ros::Time::now();
  state.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  
  this->pub_joint_trajectory_.publish(in);
  // this->pub_joint_states_.publish(state);

  // Reply back to the controller if the sender requested it.
/*  if (CommTypes::SERVICE_REQUEST == in.getCommType())
  {
    SimpleMessage reply;
    in.toReply(reply, rtn ? ReplyTypes::SUCCESS : ReplyTypes::FAILURE);
    this->getConnection()->sendMsg(reply);
  }
*/
}


}
}
