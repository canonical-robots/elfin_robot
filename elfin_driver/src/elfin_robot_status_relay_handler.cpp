/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
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

#include "elfin_driver/elfin_robot_status_relay_handler.h"
#include "industrial_robot_client/robot_status_relay_handler.h"
#include "industrial_msgs/RobotStatus.h"
#include "simple_message/log_wrapper.h"

using namespace industrial::shared_types;
using namespace industrial::smpl_msg_connection;
using namespace industrial::simple_message;
using namespace industrial::robot_status;
using namespace industrial::robot_status_message;

namespace elfin_driver
{
namespace elfin_robot_status_relay_handler
{

bool ElfinRobotStatusRelayHandler::init()
{
  sub_robot_status_ = node_.subscribe("/elfin_ros_control/elfin/enable_state", 1000, &ElfinRobotStatusRelayHandler::servoEnabledCB, this);
  return true;
}

bool ElfinRobotStatusRelayHandler::internalCB(SimpleMessage & in)
{
	ROS_ERROR("Not implemented");
	return false;
}

void ElfinRobotStatusRelayHandler::servoEnabledCB(const std_msgs::Bool & in)
{
  status_.servos = in.data;

  industrial_msgs::RobotStatus status;
//  bool rtn = true;

  status.header.stamp = ros::Time::now();
  status.drives_powered.val = status_.servos ? industrial_msgs::TriState().TRUE : industrial_msgs::TriState().FALSE;
/*  status.e_stopped.val = TriStates::toROSMsgEnum(in.status_.getEStopped());
  status.error_code = in.status_.getErrorCode();
  status.in_error.val = TriStates::toROSMsgEnum(in.status_.getInError());
  status.in_motion.val = TriStates::toROSMsgEnum(in.status_.getInMotion());
  status.mode.val = RobotModes::toROSMsgEnum(in.status_.getMode());
  status.motion_possible.val = TriStates::toROSMsgEnum(in.status_.getMotionPossible());
  */
  this->pub_robot_status_.publish(status);

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
