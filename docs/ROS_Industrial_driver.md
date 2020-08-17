
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
		sleep(1); // This loop waits up to 20 seconds to complete the trajectory
		// Place code to monitor the trajectory
	}
	spinner.stop();

}
```
The driver publishes the status of the robot on the /robot_status topic.
```c++
	ros::Subscriber sub_status_ = n->subscribe("robot_status", 1000, status_trajectory);
```
The [received message](http://docs.ros.org/melodic/api/industrial_msgs/html/msg/RobotStatus.html) is of type:
```c++
  industrial_msgs::RobotStatus status
```
The status should be used to learn about the robot state previously to command a trajectory. From the message fields: 
```c++
  status.header.stamp; // Message timestamo
  status.drives_powered.val; // True if the robot is powered on. False otherwise.
  status.in_error.val; // True if there is an error in the robot. False otherwise.
  status.in_motion.val; // True is the robot is performing a trayectory. False otherwise.  
  status.mode.val; // Not used.
  status.motion_possible.val; //True if the robot is waiting to perform a trajectory. False otherwise
  status.e_stopped.val; //True if the robot is stopped duw to an error. False otherwise;
  status.error_code; // Takes value -1 if there is an error in the robot or 0 if there is no error.
```
To know about the position of robot, while stopped and while performing a trajectory, the position is provided throug the /feedback_states topic.
```c++
	ros::Subscriber sub_joint_trajectory_ = n->subscribe("/feedback_states", 1, trajectory_trajectory);
```
The [received message](http://docs.ros.org/electric/api/control_msgs/html/msg/FollowJointTrajectoryFeedback.html) is of type:
```c++
  control_msgs::FollowJointTrajectoryFeedback
```
If the desired and actual fields are not equal, then a trajectory is being performed. The desired field is the target position of the trajectory and the actual field the current location of each joint. Following ROS conventions, the joints are numbered from the base to the TCP (Tool Control Point).

To perform a trajectory it is needed to use the joint_path_command service.

```c++
	ros::ServiceClient client = n->serviceClient<industrial_msgs::CmdJointTrajectory>("joint_path_command");
```
To call the service, a 
```c++
    industrial_msgs::CmdJointTrajectory 
```
is needed, containing [joints names and several points of a trajectory] (http://docs.ros.org/kinetic/api/trajectory_msgs/html/msg/JointTrajectory.html). Currently only one point is supported. It is possible to create complex trajectories calling the service each time.

To setup a CmdJointTrajectory:
```c++
	trajectory_msgs::JointTrajectoryPoint target;
	target.accelerations = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	target.velocities = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	target.positions = {2.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	target.effort = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

	industrial_msgs::CmdJointTrajectory srv; 
	
	srv.request.trajectory.joint_names = {"elfin_joint1", "elfin_joint2", "elfin_joint3", "elfin_joint4", "elfin_joint5","elfin_joint6"};
	srv.request.trajectory.points = {target};
```
And finally, call the service:

```c++
    client.call(srv)
```
Any error during the trajectory will be reported on the RobotStatus messages.

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
