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
#include "industrial_msgs/RobotMode.h"
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
  // Susbscribed topics from non ROS-Industrial driver
  sub_robot_status_ = node_.subscribe("/elfin_ros_control/elfin/enable_state", 1000, &ElfinRobotStatusRelayHandler::servoEnabledCB, this);
  sub_robot_fault_ = node_.subscribe("/elfin_ros_control/elfin/fault_state", 1000, &ElfinRobotStatusRelayHandler::faultCB, this);
  sub_arm_controller_ = node_.subscribe("/elfin_arm_controller/state", 1000, &ElfinRobotStatusRelayHandler::armControllerCB, this);
  
  // Published topic for ROS-Industrial driver
  pub_robot_status_ = node_.advertise<industrial_msgs::RobotStatus>("robot_status", 1);
  
  // Service from non ROS-Industrial driver
  get_motion_state_client_ = node_.serviceClient<std_srvs::SetBool>("/elfin_ros_control/elfin/get_motion_state");
  
  return true;
}

bool ElfinRobotStatusRelayHandler::internalCB(SimpleMessage & in)
{
	ROS_ERROR("Not implemented");
	return false;
}

void ElfinRobotStatusRelayHandler::armControllerCB(const std_msgs::Bool & in)
{
	
}

void ElfinRobotStatusRelayHandler::faultCB(const std_msgs::Bool & in)
{
	status_.fault = in.data;
}

void ElfinRobotStatusRelayHandler::servoEnabledCB(const std_msgs::Bool & in)
{
  status_.servos = in.data;

  industrial_msgs::RobotStatus status;
//  bool rtn = true;

  status.header.stamp = ros::Time::now();
  status.drives_powered.val = status_.servos ? industrial_msgs::TriState().TRUE : industrial_msgs::TriState().FALSE;
  status.in_error.val = status_.fault ? industrial_msgs::TriState().TRUE : industrial_msgs::TriState().FALSE;
  status.in_motion.val = checkMotionState() ? industrial_msgs::TriState().TRUE : industrial_msgs::TriState().FALSE;
  
  status.mode.val = industrial_msgs::RobotMode().UNKNOWN;
  status.motion_possible.val = !status_.fault ? industrial_msgs::TriState().TRUE : industrial_msgs::TriState().FALSE;
  status.e_stopped.val = status_.fault ? industrial_msgs::TriState().TRUE : industrial_msgs::TriState().FALSE;
  status.error_code = status_.fault ? -1 : 0;
  
/*  status.e_stopped.val = TriStates::toROSMsgEnum(in.status_.getEStopped());
  status.error_code = in.status_.getErrorCode();
  status.in_error.val = TriStates::toROSMsgEnum(in.status_.getInError());
  status.in_motion.val = TriStates::toROSMsgEnum(in.status_.getInMotion());
  status.mode.val = RobotModes::toROSMsgEnum(in.status_.getMode());
  status.motion_possible.val = TriStates::toROSMsgEnum(in.status_.getMotionPossible());
  */
  ROS_INFO("el resultado: %d", in.data);
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

bool ElfinRobotStatusRelayHandler::checkMotionState()
{
    std_srvs::SetBool::Request req_tmp;
    std_srvs::SetBool::Response resp_tmp;

    // Check motion state
    if(!get_motion_state_client_.exists())
    {
        return false;
    }

    req_tmp.data=true;
    get_motion_state_client_.call(req_tmp, resp_tmp);
    if(resp_tmp.success)
    {
        return true;
    }
	else
		return false;
}

}
}
