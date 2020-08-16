/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, 2020 Canonical Robots
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Canonical Robots, nor the names
 *      of its contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
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
 
#include "control_msgs/JointTrajectoryControllerState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "industrial_msgs/RobotStatus.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

std_msgs::Bool enable_state_msg_;
std_msgs::Bool fault_state_msg_;
control_msgs::JointTrajectoryControllerState joint_trajectory_state_msg_;
trajectory_msgs::JointTrajectoryPoint actual_;
trajectory_msgs::JointTrajectoryPoint desired_;
trajectory_msgs::JointTrajectoryPoint error_;
float velocity = 0.1f;

bool comparePositions(const std::vector<double> a, const std::vector<double> b)
{
	ROS_DEBUG("Comparing...");
  int ii;
  for(ii = 0; ii < 6; ii++) {
	  ROS_DEBUG("Comparing %d: %f - %f", ii, b[ii], a[ii] );
    if(!(fabs(a[ii]-b[ii]) < 1e-10 	)){ ROS_DEBUG("Not equal!! %f - %f", b[ii], a[ii]); return false;		};
  }
  ROS_DEBUG("Equal!!");
  return true;
}

void moveRobot(std::vector<double> a, 	std::vector<double> &b)
{
  int ii;
  ROS_DEBUG("Moving...");
  for(ii = 0; ii < 6; ii++) {

    if(fabs(b[ii]-a[ii])-velocity < 1e-10 * (fabs(a[ii]) + fabs(b[ii]))) 
	{
		ROS_DEBUG("Reached on joint %d", ii);
		b[ii]=a[ii];
	}
	else if(b[ii]-a[ii]>0) {b[ii]=b[ii]-velocity;}
		  else {b[ii]=b[ii]+velocity;}
	// Some fault condition... i.e. any value > |3,14|
	if(fabs(b[ii])> 3.14159)
	{
		enable_state_msg_.data = false;
		fault_state_msg_.data = true;
	}
  }
}

bool trajectory_received = false;

void jointTrajectoryCB(const trajectory_msgs::JointTrajectory &msg)
{
    ROS_DEBUG("Trajectory received");
	trajectory_received = true;
	enable_state_msg_.data = true;
	fault_state_msg_.data = false;
	for(int ii = 0; ii < 6; ii++) {
		ROS_DEBUG("Trajectory received %f", msg.points[0].positions[ii]);
	}
	joint_trajectory_state_msg_.desired.positions = msg.points[0].positions;
    ROS_DEBUG("Trajectory processed");
}

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "elfin_robot_mockup");
  
  ros::NodeHandle n;
  
  ros::Publisher pub_robot_status_ = n.advertise<std_msgs::Bool>("/elfin_ros_control/elfin/enable_state", 1000);
  ros::Publisher pub_robot_fault_ = n.advertise<std_msgs::Bool>("/elfin_ros_control/elfin/fault_state", 1000);
  ros::Publisher pub_arm_controller_ = n.advertise<control_msgs::JointTrajectoryControllerState>("/elfin_arm_controller/state", 1000);
  ros::Subscriber sub_joint_trajectory_ = n.subscribe("elfin_arm_controller/command", 1000, jointTrajectoryCB);

  ros::Rate loop_rate(10);
  enable_state_msg_.data = true;
  fault_state_msg_.data = false;
  actual_.accelerations = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  actual_.velocities = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  actual_.positions = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  actual_.effort = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  desired_.accelerations = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  desired_.velocities = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  desired_.positions = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  desired_.effort = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  error_.accelerations = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  error_.velocities = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  error_.positions = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  error_.effort = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  
  joint_trajectory_state_msg_.actual = actual_;
  joint_trajectory_state_msg_.desired = desired_;
  joint_trajectory_state_msg_.error = error_;
  
  
  while (true)
  {
	pub_robot_status_.publish(enable_state_msg_);
	pub_robot_fault_.publish(fault_state_msg_);
	if(trajectory_received)
	{
		if(!comparePositions(joint_trajectory_state_msg_.desired.positions, joint_trajectory_state_msg_.actual.positions)){
			moveRobot(joint_trajectory_state_msg_.desired.positions, joint_trajectory_state_msg_.actual.positions);
		}
		else 
		{
			ROS_DEBUG("Trajectory reached!!");
			trajectory_received = false;
		}
	}
	pub_arm_controller_.publish(joint_trajectory_state_msg_);			
	ros::spinOnce();
	loop_rate.sleep();
  }

  return 0;
}
