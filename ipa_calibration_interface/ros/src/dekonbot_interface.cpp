/****************************************************************
 *
 * Copyright (c) 2021
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: ipa_calibration
 * ROS stack name: ipa_calibration
 * ROS package name: ipa_calibration_interface
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Marc Riedlinger, email: m.riedlinger@live.de
 * Author: Florian Jordan, email: florian.jordan@ipa.fraunhofer.de
 *
 * Date of creation: July 2021
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <ipa_calibration_interface/dekonbot_interface.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>


DekonbotInterface::DekonbotInterface(ros::NodeHandle* nh, CalibrationType* calib_type, CalibrationMarker* calib_marker, bool do_arm_calibration, bool load_data) :
				IPAInterface(nh, calib_type, calib_marker, do_arm_calibration, load_data), arm_state_current_(0), camera_state_current_(0),
				arm_action_client_("/arm/joint_trajectory_controller/follow_joint_trajectory", true),
				camera_action_client_("/torso/joint_trajectory_controller/follow_joint_trajectory", true)
{
	std::cout << "\n========== DeKonbot Interface Parameters ==========\n";

	// Adjust here: Add all needed code in here to let robot move itself, its camera and arm.
	node_handle_.param<std::string>("camera_joint_controller_command", camera_joint_controller_command_, "");
	std::cout << "camera_joint_controller_command: " << camera_joint_controller_command_ << std::endl;
	camera_joint_controller_ = node_handle_.advertise<std_msgs::Float64MultiArray>(camera_joint_controller_command_, 1, false);

	camera_state_current_ = new sensor_msgs::JointState;
	if ( camera_state_current_ != 0 )
	{
		node_handle_.param<std::string>("camera_joint_state_topic", camera_joint_state_topic_, "");
		std::cout << "camera_joint_state_topic: " << camera_joint_state_topic_ << std::endl;
		camera_state_ = node_handle_.subscribe<sensor_msgs::JointState>(camera_joint_state_topic_, 0, &DekonbotInterface::cameraStateCallback, this);
		camera_action_client_.waitForServer();
	}
	else
	{
		ROS_ERROR("DekonbotInterface::DekonbotInterface - Could not create current camera state storage!");
		return;
	}

	if ( arm_calibration_ )
	{
		ROS_ERROR("DekonbotInterface::DekonbotInterface - Arm Calibration not supported!");
		return;

		node_handle_.param<std::string>("arm_joint_controller_command", arm_joint_controller_command_, "");
		std::cout << "arm_joint_controller_command: " << arm_joint_controller_command_ << std::endl;
		arm_joint_controller_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(arm_joint_controller_command_, 1, false);

		arm_state_current_ = new sensor_msgs::JointState;
		if ( arm_state_current_ != 0 )
		{
			node_handle_.param<std::string>("arm_state_topic", arm_state_topic_, "");
			std::cout << "arm_state_topic: " << arm_state_topic_ << std::endl;
			arm_state_ = node_handle_.subscribe<sensor_msgs::JointState>(arm_state_topic_, 0, &DekonbotInterface::armStateCallback, this);
			arm_action_client_.waitForServer();
		}
	}
	else
	{
		node_handle_.param<std::string>("base_controller_topic_name", base_controller_topic_name_, "");
		std::cout << "base_controller_topic_name: " << base_controller_topic_name_ << std::endl;
		base_controller_ = node_handle_.advertise<geometry_msgs::Twist>(base_controller_topic_name_, 1, false);
	}

	ROS_INFO("DekonbotInterface::DekonbotInterface - DekonbotInterface initialized.");
}

DekonbotInterface::~DekonbotInterface()
{
	if ( camera_state_current_ != 0 )
		delete camera_state_current_;

	if ( arm_state_current_ != 0 )
		delete arm_state_current_;
}


// CAMERA CALIBRATION INTERFACE

//Callbacks - User defined
void DekonbotInterface::cameraStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	boost::mutex::scoped_lock lock(camera_state_data_mutex_);
	*camera_state_current_ = *msg;
}

void DekonbotInterface::armStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	boost::mutex::scoped_lock lock(arm_state_data_mutex_);
	*arm_state_current_ = *msg;
}
// End Callbacks

void DekonbotInterface::assignNewRobotVelocity(geometry_msgs::Twist new_velocity) // Spin and move velocities
{
	// In current DeKonBot setup, arm is placed 180 degrees rotated (along z axis), so change sign of linear velocity
	new_velocity.linear.x *= -1.0;
	base_controller_.publish(new_velocity);
}

void DekonbotInterface::assignNewCameraAngles(const std::string &camera_name, std_msgs::Float64MultiArray new_camera_angles)
{
	// Adjust here: Assign new camera tilt angle here
	trajectory_msgs::JointTrajectoryPoint joint_traj_point;
	trajectory_msgs::JointTrajectory joint_traj;
	control_msgs::FollowJointTrajectoryGoal cam_goal;

	if ( camera_name.compare("realsense_sr300") == 0 )
		joint_traj.joint_names = {"arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_joint", "arm_wrist_1_joint", "arm_wrist_2_joint",
									 "arm_wrist_3_joint"};
	else if ( camera_name.compare("ids_ueye") )
		joint_traj.joint_names = {"torso_bottom_joint", "torso_side_joint"};
	else
	{
		ROS_ERROR("DekonbotInterface::assignNewCameraAngles - Invalid camera name %s, cannot move anything!", camera_name.c_str());
		return;
	}

	joint_traj_point.positions.insert(joint_traj_point.positions.end(), new_camera_angles.data.begin(), new_camera_angles.data.end());
	joint_traj_point.time_from_start = ros::Duration(2);
	joint_traj_point.velocities = {0,0}; //Initialize velocities to zero, does not work with empty list
	joint_traj_point.accelerations = {0,0};
	joint_traj.points.push_back(joint_traj_point);
	joint_traj.header.stamp = ros::Time::now();

	cam_goal.trajectory = joint_traj;
	camera_action_client_.sendGoal(cam_goal);
}

std::vector<double>* DekonbotInterface::getCurrentCameraState(const std::string &camera_name)
{
	boost::mutex::scoped_lock lock(camera_state_data_mutex_);
	return &camera_state_current_->position;
}
// END CALIBRATION INTERFACE


// ARM CALIBRATION INTERFACE
void DekonbotInterface::assignNewArmJoints(const std::string &arm_name, std_msgs::Float64MultiArray new_arm_config)
{
	// Adjust here: Assign new joints to your robot arm
	trajectory_msgs::JointTrajectoryPoint joint_traj_point;
	trajectory_msgs::JointTrajectory joint_traj;
	control_msgs::FollowJointTrajectoryGoal arm_goal;
	joint_traj.joint_names = {"arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_joint", "arm_wrist_1_joint", "arm_wrist_2_joint",
							 "arm_wrist_3_joint"};
	joint_traj_point.positions.insert(joint_traj_point.positions.end(), new_arm_config.data.begin(), new_arm_config.data.end());
	joint_traj_point.time_from_start = ros::Duration(2); // Quick and dirty, should actually be calculated by max angular speed and travel distance
	joint_traj_point.velocities = {0,0,0,0,0,0};
	joint_traj_point.accelerations = {0,0,0,0,0,0};
	joint_traj.points.push_back(joint_traj_point);
	joint_traj.header.stamp = ros::Time::now();
	arm_goal.trajectory = joint_traj;

	arm_action_client_.sendGoal(arm_goal);
}

std::vector<double>* DekonbotInterface::getCurrentArmState(const std::string &arm_name)
{
	boost::mutex::scoped_lock lock(arm_state_data_mutex_);
	return &arm_state_current_->position;
}
// END

std::string DekonbotInterface::getRobotName()
{
	return "RAW-3-1";
}




