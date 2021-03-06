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

#include "elfin_driver/elfin_robot_state_interface.h"
#include "industrial_utils/param_utils.h"

using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial_utils::param::getJointNames;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

namespace elfin_driver
{
namespace elfin_robot_state_interface
{

ElfinRobotStateInterface::ElfinRobotStateInterface()
{
  this->connection_ = NULL;
  this->add_handler(&default_joint_handler_);
  this->add_handler(&default_robot_status_handler_);
}

bool ElfinRobotStateInterface::init()
{
  return true;
}

bool ElfinRobotStateInterface::init(std::string default_ip, int default_port)
{
  // Using ros_control instead of Simple Message
  ROS_ERROR("Not implemented");
  return false;
}

bool ElfinRobotStateInterface::init(SmplMsgConnection* connection)
{
  // Using ros_control instead of Simple Message
  ROS_ERROR("Not implemented");
  return false;
}

bool ElfinRobotStateInterface::init(SmplMsgConnection* connection, std::vector<std::string>& joint_names)
{
  // Using ros_control instead of Simple Message
  ROS_ERROR("Not implemented");
  return false;
}


} // elfin_robot_state_interface
} // elfin_driver
