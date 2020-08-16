/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
 * Copyright (c) 2019, 2020 Canonical Robots
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


#ifndef ELFIN_ROBOT_STATE_INTERFACE_H
#define ELFIN_ROBOT_STATE_INTERFACE_H

#include "simple_message/smpl_msg_connection.h"
#include "industrial_robot_client/robot_state_interface.h"
#include "elfin_driver/elfin_robot_status_relay_handler.h"

namespace elfin_driver
{
namespace elfin_robot_state_interface
{
using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial::message_manager::MessageManager;
using industrial::message_handler::MessageHandler;
using industrial_robot_client::joint_relay_handler::JointRelayHandler;
using industrial_robot_client::robot_state_interface::RobotStateInterface;
using elfin_driver::elfin_robot_status_relay_handler::ElfinRobotStatusRelayHandler;


/**
 * \brief Generic template that reads state-data from a robot controller
 * and publishes matching messages to various ROS topics.
 *
 * Users should replace the default class members
 * to implement robot-specific behavior.
 */
//* ElfinRobotStateInterface
class ElfinRobotStateInterface : public RobotStateInterface
{

public:

  /**
   * \brief Default constructor.
   */
  ElfinRobotStateInterface();

  /**
   * \brief Initialize robot connection using default method.
   *
   * \return true on success, false otherwise
   */
  bool init();  
  
  /**
   * \brief Initialize robot connection using default method.
   *
   * \param default_ip default IP address to use for robot connection [OPTIONAL]
   *                    - this value will be used if ROS param "robot_ip_address" cannot be read
   * \param default_port default port to use for robot connection [OPTIONAL]
   *                    - this value will be used if ROS param "~port" cannot be read
   *
   * \return true on success, false otherwise
   */
  bool init(std::string default_ip, int default_port);

  /**
   * \brief Initialize robot connection using specified method.
   *
   * \param connection new robot-connection instance (ALREADY INITIALIZED).
   *
   * \return true on success, false otherwise
   */
  bool init(SmplMsgConnection* connection);

  /**
   * \brief Initialize robot connection using specified method and joint-names.
   *
   * \param connection new robot-connection instance (ALREADY INITIALIZED).
   * \param joint_names list of joint-names for ROS topic
   *   - Count and order should match data sent to robot connection.
   *   - Use blank-name to skip (not publish) a joint-position
   *
   * \return true on success, false otherwise
   */
  bool init(SmplMsgConnection* connection, std::vector<std::string>& joint_names);


  /**
   * \brief get current robot-connection instance.
   *
   * \return current robot connection object
   */
  SmplMsgConnection* get_connection()
  {
    return NULL;
  }

protected:

	ElfinRobotStatusRelayHandler default_robot_status_handler_;
  
};//class ElfinRobotStateInterface

}//elfin_robot_state_interface
}//elfin_robot_driver


#endif /* ELFIN_ROBOT_STATE_INTERFACE_H */
