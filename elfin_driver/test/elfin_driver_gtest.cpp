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
#include "industrial_msgs/RobotStatus.h"
#include "sensor_msgs/JointState.h"


#include <stdio.h>
#include <stdlib.h>
#include <gtest/gtest.h>

using namespace industrial::robot_status;
using namespace industrial::robot_status_message;


char path[256];


TEST(ElfinDriver, launch)
{
  char cmd [512];
  if(sizeof(path)> 450){
	  EXPECT_EQ(1, 0);
	  return;
  }
  // lauching the driver from ROS
  strncpy(cmd, path, sizeof(path));
  strcat(cmd, "elfin_launch.sh");
  printf("%s\n", cmd);
  system(cmd);
  
  memset(cmd,0,sizeof(cmd)); 
  
  // launching verification scripts
  strncpy(cmd, path, sizeof(path));
  strcat(cmd, "elfin_robot_status.sh");
  // Verifying the enabled state and aliases values match
  printf("%s\n", cmd);
  int x = system(cmd);
  EXPECT_EQ(1, WEXITSTATUS(x));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");  // some tests need ROS framework
  testing::InitGoogleTest(&argc, argv);
  char *p =strrchr(argv[0], '/');
  strncpy(path, argv[0], p - argv[0] + 1);
  

  return RUN_ALL_TESTS();
}