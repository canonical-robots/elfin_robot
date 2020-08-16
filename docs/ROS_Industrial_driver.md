
Elfin Robot ROS Industrial Driver
======

The detailed API for a ROS Industrial Driver is described [here](http://wiki.ros.org/Industrial/Industrial_Robot_Driver_Spec).

This document illustrates how to use the ROS Industrial Driver implementation for an Elfin Robot.

Starting from this sample code:

```c++
#include "control_msgs/FollowJointTrajectoryFeedback.h"
#include "elfin_driver/elfin_robot_status_relay_handler.h"
#include "industrial_msgs/RobotStatus.h"
#include "sensor_msgs/JointState.h"
#include "industrial_msgs/RobotMode.h"
#include "industrial_msgs/CmdJointTrajectory.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"

int main(int argc, char **argv) {
	int i = 0;
	trajectory_received = false;
	ros::Subscriber sub_status_ = n->subscribe("robot_status", 1000, status_trajectoryTest);
	ros::Subscriber sub_joint_trajectory_ = n->subscribe("/feedback_states", 1, trajectory_trajectoryTest);
	ros::ServiceClient client = n->serviceClient<industrial_msgs::CmdJointTrajectory>("joint_path_command");

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
		i++;
	}
	spinner.stop();

}
```

## Acknowledgement

This fork, including its modifications, has received funding from ROSIN project.

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a></br>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287.
