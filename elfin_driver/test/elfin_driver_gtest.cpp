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


#include "control_msgs/FollowJointTrajectoryFeedback.h"
#include "elfin_driver/elfin_robot_status_relay_handler.h"
#include "industrial_msgs/RobotStatus.h"
#include "sensor_msgs/JointState.h"
#include "industrial_msgs/RobotMode.h"
#include "industrial_msgs/CmdJointTrajectory.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "industrial_msgs/StopMotion.h"
#include "sensor_msgs/JointState.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <gtest/gtest.h>

using namespace industrial::robot_status;
using namespace industrial::robot_status_message;


char path[256];
ros::NodeHandle* n = NULL;
industrial_msgs::RobotStatus status;
trajectory_msgs::JointTrajectoryPoint current;
bool trajectory_received = false;

//-- Test status callbacks --
void statusTest(const industrial_msgs::RobotStatus & in)
{
	status = in;
	
}

//-- Test trajectory_api callbacks --
void trajectory_apiTest(const control_msgs::FollowJointTrajectoryFeedback & in)
{
	ROS_DEBUG("Trajectory received");
	trajectory_received = true;
	current.positions = in.actual.positions;
}

//-- Test trajectory callbacks --
void status_trajectoryTest(const industrial_msgs::RobotStatus & in)
{
	status = in;
}

void trajectory_trajectoryTest(const control_msgs::FollowJointTrajectoryFeedback & in)
{
	ROS_DEBUG("Trajectory received");
	trajectory_received = true;
	current.positions = in.actual.positions;
}

//-- Test wrong_trajectory callbacks --
void status_wrong_trajectoryTest(const industrial_msgs::RobotStatus & in)
{
	status = in;
}

void trajectory_wrong_trajectoryTest(const control_msgs::FollowJointTrajectoryFeedback & in)
{
	ROS_DEBUG("Trajectory received");
	trajectory_received = true;
	current.positions = in.actual.positions;
}

//-- Utils --
bool comparePositions(const std::vector<double> a, const std::vector<double> b)
{
  int ii;
  for(ii = 0; ii < 6; ii++) {
	  ROS_DEBUG("Comparing %d: %f - %f", ii, b[ii], a[ii] );
    if(!(fabs(a[ii]-b[ii]) < 1e-10 	)){ ROS_DEBUG("Not equal!! %f - %f", b[ii], a[ii]); return false;		};
  }
  return true;
}

//-- Unit Tests --
TEST(ElfinDriver_unit, started)
{
  ros::Subscriber sub_status_ = n->subscribe("robot_status", 1, statusTest);
  ros::AsyncSpinner spinner(1); 
  spinner.start();
  sleep(2);
  EXPECT_EQ(0, status.error_code);
  spinner.stop();
}

TEST(ElfinDriver_unit, trajectory_api)
{
  ros::Subscriber sub_status_ = n->subscribe("feedback_states", 1000, trajectory_trajectoryTest);
  ros::AsyncSpinner spinner(1); 
  spinner.start();
  sleep(2);
  EXPECT_TRUE(trajectory_received) << "No trajectory received";
  spinner.stop();
}

TEST(ElfinDriver_unit, service_api)
{
	ros::ServiceClient client = n->serviceClient<industrial_msgs::CmdJointTrajectory>("joint_path_command");
	sleep(1);
	trajectory_msgs::JointTrajectoryPoint target;
	target.accelerations = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	target.velocities = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	target.positions = {2.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	target.effort = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

	industrial_msgs::CmdJointTrajectory srv; 
	
	srv.request.trajectory.joint_names = {"elfin_joint1", "elfin_joint2", "elfin_joint3", "elfin_joint4", "elfin_joint5","elfin_joint6"};
	srv.request.trajectory.points = {target};
	
	EXPECT_TRUE(client.call(srv)) << "Error calling 'joint_path_command' service";
}

//-- Integration Tests --
TEST(ElfinDriver_integration, trajectory)
{
	int i = 0;
	trajectory_received = false;
	ros::Subscriber sub_status_ = n->subscribe("robot_status", 1000, status_trajectoryTest);
	ros::Subscriber sub_joint_trajectory_ = n->subscribe("/feedback_states", 1, trajectory_trajectoryTest);
	ros::ServiceClient client = n->serviceClient<industrial_msgs::CmdJointTrajectory>("joint_path_command");
	sleep(1);
	trajectory_msgs::JointTrajectoryPoint target;
	target.accelerations = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	target.velocities = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	target.positions = {2.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	target.effort = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

	industrial_msgs::CmdJointTrajectory srv; 
	ros::AsyncSpinner spinner(1); 
	spinner.start();
	
	srv.request.trajectory.joint_names = {"elfin_joint1", "elfin_joint2", "elfin_joint3", "elfin_joint4", "elfin_joint5","elfin_joint6"};
	srv.request.trajectory.points = {target};


	if (!client.call(srv))
    {
		ROS_ERROR("Error calling service.");
		ASSERT_TRUE(false);
    }
	ROS_DEBUG("Service called");
	while(!	trajectory_received && i<5)
	{
		ROS_DEBUG("No trajectory %d", i);
		sleep(1);
		i++;	
	}
	i=0;
	while(trajectory_received && !comparePositions(current.positions, target.positions) && i < 20)
	{
		sleep(1);
		i++;
	}
	spinner.stop();
	ASSERT_TRUE(trajectory_received) << "No trajectory received";
    EXPECT_LT(i, 20);
}

TEST(ElfinDriver_integration, wrong_trajectory)
{
	int i = 0;
	trajectory_received = false;
	ros::Subscriber sub_status_ = n->subscribe("robot_status", 1000, status_wrong_trajectoryTest);
	ros::Subscriber sub_joint_trajectory_ = n->subscribe("/feedback_states", 1, trajectory_wrong_trajectoryTest);
	ros::ServiceClient client = n->serviceClient<industrial_msgs::CmdJointTrajectory>("joint_path_command");
	sleep(1);
	trajectory_msgs::JointTrajectoryPoint target;
	target.accelerations = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	target.velocities = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	target.positions = {4.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	target.effort = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

	industrial_msgs::CmdJointTrajectory srv; 
	ros::AsyncSpinner spinner(1); 
	spinner.start();
	
	srv.request.trajectory.joint_names = {"elfin_joint1", "elfin_joint2", "elfin_joint3", "elfin_joint4", "elfin_joint5","elfin_joint6"};
	srv.request.trajectory.points = {target};


	if (!client.call(srv))
    {
		ROS_ERROR("Error calling service.");
		ASSERT_TRUE(false);
    }
	ROS_DEBUG("Service called");
	while(!	trajectory_received && i<5)
	{
		ROS_DEBUG("No trajectory %d", i);
		ros::spinOnce();
		sleep(1);
		i++;	
	}
	i=0;
	while(trajectory_received && !comparePositions(current.positions, target.positions) && i < 20)
	{
		sleep(1);
		i++;
	}
	spinner.stop();
	ASSERT_TRUE(trajectory_received) << "No trajectory received";
    EXPECT_LT(i, 20);
	EXPECT_EQ(-1, status.error_code);
} 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");  // some tests need ROS framework  
  n = new ros::NodeHandle();

  testing::InitGoogleTest(&argc, argv);
  char *p =strrchr(argv[0], '/');
  strncpy(path, argv[0], p - argv[0] + 1);
  

  return RUN_ALL_TESTS();
}
